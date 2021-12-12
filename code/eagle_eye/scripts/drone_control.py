#!/usr/bin/env python3
import sys
from collections import namedtuple

from numpy.core.numeric import True_
import rospy
from rospy.impl.tcpros import get_tcpros_handler
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3Stamped, Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import clover.srv

import cv2, numpy as np
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.spatial import KDTree

### Probabilistic Masking (Null Hypothesis test) ###
import scipy.stats


def pval(dist, points):
    "Returns the p-value of points on a symmetric distribution."
    cumul = dist.cdf(points)
    return 2*np.minimum(cumul, 1-cumul)


def pval_lower(dist, points):
    "Returns the lower p-value of points on a distribution."
    return dist.cdf(points)


def pval_upper(dist, points):
    "Returns the upper p-value of points on a distribution."
    return dist.sf(points)


def recenter_hue(hue, mid_hue):
    return np.mod(hue - mid_hue + 90, 180) + mid_hue - 90


RED_FILTER = {
    'hue_mean': 0,
    'hue_std': 5,
    'sat_offs': 0,
    'sat_std': -5,
    'val_offs': 20,
    'val_std': -50,
}

YELLOW_FILTER = {
    'hue_mean': 29,
    'hue_std': 5,
    'sat_offs': 0,
    'sat_std': -140,
    'val_offs': 78,
    'val_std': -60,
}


class Filter:
    def __init__(self, hue_mean=0, hue_std=5, sat_offs=0, sat_std=5, val_offs=0, val_std=5):
        self.hue_dist = scipy.stats.norm(hue_mean, hue_std)
        self.sat_dist = scipy.stats.expon(sat_offs, np.abs(sat_std))
        self.val_dist = scipy.stats.expon(val_offs, np.abs(val_std))
        self.sat_inv = sat_std < 0
        self.val_inv = val_std < 0

    def apply_hue(self, hue):
        return pval(self.hue_dist, recenter_hue(hue, self.hue_dist.mean()))

    def apply_sat(self, sat):
        return pval_upper(self.sat_dist, 255 - sat if self.sat_inv else sat)

    def apply_val(self, val):
        return pval_upper(self.val_dist, 255 - val if self.val_inv else val)

    def apply(self, hsv):
        hue, sat, val = np.moveaxis(hsv.astype(float), -1, 0)
        pval_hue = self.apply_hue(hue)
        pval_sat = self.apply_sat(sat)
        pval_val = self.apply_val(val)
        return pval_hue * pval_sat * pval_val

### Coordinate Transformations ###

WIDTH = 320
HEIGHT = 240

def idx_to_xy(idx):
    return idx[...,::-1] * (1,-1) + (0, HEIGHT)

def xy_to_idx(xy):
    return ((xy - (0, HEIGHT)) * (1,-1))[...,::-1]
    
def to_cv(idx):
    return np.round(idx[...,::-1]).astype(int).clip(-99999, 99999)

def turn_left(xy):
    x, y = np.moveaxis(xy, -1 , 0)
    return np.stack([-y, x], axis=-1)

def to_clover(xy):
    x, y = np.moveaxis(xy, -1 , 0)
    return np.stack([y, -x], axis=-1)

### Computer Vision ###

RESOLUTION = 100 # pixels / meter

# https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html
def undistort_plumb_bob(camera_matrix, distortion, image_size, alpha=1):
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion, image_size, alpha, image_size)
    def undistort(img):
        img_dst = cv2.undistort(img, camera_matrix, distortion, None, new_camera_matrix)
        #x, y, w, h = roi
        #img_dst = img_dst[y:y+h, x:x+w]
        return img_dst
    return undistort

def parse_telemetry(msg):
    rotation = np.array([msg.pitch, msg.roll, msg.yaw])
    translation = np.array([msg.x, msg.y, msg.z])
    velocity = np.array([msg.vx, msg.vy, msg.vz])
    return rotation, translation, velocity

# https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
project_hom = np.array(
    [[1,0,0],
     [0,1,0],
     [0,0,0],
     [0,0,1]])

def homogeneous_gnd2cam(R_euler, T_vec, K):
    rotation = Rotation.from_euler('xyz', R_euler)
    R = rotation.as_matrix()
    T = np.concatenate([np.eye(3), T_vec[:,None]], axis=-1)
    M = R @ T @ project_hom
    H = K @ M
    return H

def homogeneous_cam2gnd(R_euler, T_vec, K_inv):
    rotation = Rotation.from_euler('xyz', R_euler)
    R = rotation.as_matrix()
    R_inv = R.T
    T_vec_inv = R_inv @ T_vec
    M_inv = np.concatenate([R_inv, T_vec_inv[:,None]], axis=-1)
    H_inv = M_inv @ project_hom @ K_inv
    return H_inv

def homogeneous_translation(t_vec):
    T = np.eye(len(t_vec)+1)
    T[:len(t_vec),-1] = t_vec
    return T

def apply_hom(H, pts, inv=False):
    pts_hom = cv2.convertPointsToHomogeneous(pts)[:,0,:]
    pts_tf_hom = (H @ pts_hom.T).T
    if inv:
        pts_tf = pts_tf_hom[...,:2] * pts_tf_hom[...,2,None]
    else:
        pts_tf = cv2.convertPointsFromHomogeneous(pts_tf_hom)[:,0,:]
    return pts_tf

corners_coeff = np.array(
    [[0,0],
     [0,1],
     [1,0],
     [1,1]]
)
def roi(image_size, H):
    "Returns bounding box after an image of some size goes through a homogenous transformation"
    corners = corners_coeff * image_size
    corners_tf = apply_hom(H, corners, inv=True)
    min_pt = np.min(corners_tf, axis=0)
    max_pt = np.max(corners_tf, axis=0)
    return min_pt, max_pt

def project_below(img, R_euler, T_vec, K, K_inv):
    # Create homogeneous transformation from camera to ground
    T_vec_centered = np.zeros_like(T_vec)
    T_vec_centered[2] = T_vec[2]
    H = homogeneous_gnd2cam(R_euler, T_vec_centered, K)
    H_inv = homogeneous_cam2gnd(R_euler, T_vec_centered, K_inv)

    # Scale from meters to pixels
    S = np.eye(3) * (1/RESOLUTION, 1/RESOLUTION, 1) # ground plane pixels to meters
    S_inv = np.eye(3) * (RESOLUTION, RESOLUTION, 1) # meters to ground plane pixels
    H_scaled = H @ S
    H_inv_scaled = S_inv @ H_inv

    # Apply bounding box but retain center
    min_pt, max_pt = roi(img.shape[:2][::-1], H_inv_scaled)
    max_dist = np.max(np.abs(np.stack([min_pt, max_pt])),axis=0)
    out_size = np.ceil(max_dist * 2).astype(int)
    T_center = homogeneous_translation(-out_size / 2)
    T_center_inv = homogeneous_translation(out_size / 2)
    H_inv_scaled_centered = T_center_inv @ H_scaled
    H_scaled_centered = H_scaled @ T_center

    # Apply transformation
    img_below = cv2.warpPerspective(img, H_scaled_centered, out_size, flags=cv2.WARP_INVERSE_MAP)
    return img_below, H_scaled_centered

### Image Stitching ###

def to_diag(pnts):
    "Return distance from point to diagonal and position on diagonal"
    vector = np.ones(pnts.shape[-1])
    counts = pnts.shape[-1] - np.ma.count_masked(pnts, axis=-1)
    line_len = np.sqrt(counts)
    t = np.ma.sum(pnts * vector, axis=-1) / line_len
    pt_dist_sq = np.ma.sum(pnts ** 2, axis=-1)
    dist_sq = pt_dist_sq - t**2
    return (dist_sq, t / line_len)

from numpy.lib.stride_tricks import sliding_window_view as viewW
# https://stackoverflow.com/a/51613442
def strided_indexing_roll(a, r):
    # Concatenate with sliced to cover all rolls
    a_ext = np.concatenate((a, a[...,:-1]),axis=-1)

    # Get sliding windows; use advanced-indexing to select appropriate ones
    n = a.shape[-1]
    arr_ret = viewW(a_ext,n,axis=-1)[(*np.indices(r.shape)), (n-r)%n]

    # Roll and apply mask too
    if np.ma.isMA(a):
        arr_ret = np.ma.array(arr_ret, mask=strided_indexing_roll(np.ma.getmaskarray(a), r), copy=False)
    return arr_ret

def diagonals(arr, k):
    length = np.min(arr.shape[:2])
    i = np.arange(length)
    j = (i[:,None] + k).T
    mask = (j < 0) | (j >= arr.shape[1])
    j[mask] = 0
    shp = np.concatenate([np.array(j.shape), np.array(arr.shape[2:])])
    shp_flat = shp.copy()
    shp_flat[2:] = 1
    mask_expanded = np.broadcast_to(mask.reshape(shp_flat), shp)
    return np.ma.array(arr[i,j], mask=mask_expanded)

def initialize_data(img1, img2):
    data = {'images': [{'img': img1}, {'img': img2}]}

    for d in data['images']:
        d['filt'] = d['img'].astype(np.int8)

        d['axis'] = [{'name': 'y'}, {'name': 'x'}]
        for ax, a in enumerate(d['axis']):
            a['diff'] = np.diff(d['filt'], axis=ax)

            a['dir'] = [{'dir': 1}, {'dir': -1}]
            for dr in a['dir']:
                dr['mask'] = a['diff'] == dr['dir']
                dr['cum_nonaligned'] = np.cumsum(dr['mask'], axis=ax)
                dr['cum'] = dr['cum_nonaligned'].T if a['name'] == 'y' else dr['cum_nonaligned']
                dr['count'] = dr['cum'][:,-1]
                dr['maxcount'] = np.max(dr['count'])
                offs = np.arange(dr['cum'].shape[0]) * (dr['maxcount']+1)
                cumflat = (dr['cum'] + offs[:,None]).flat
                searchfor = offs[:,None] + np.arange(dr['maxcount']) + 1
                ss = np.searchsorted(cumflat, searchfor.flat)
                offs = np.arange(dr['cum'].shape[0]) * dr['cum'].shape[1]
                dr['indices'] = ss.reshape((dr['cum'].shape[0], dr['maxcount'])) - offs[:,None]

                # Convert to masked array
                dr['indices'] = np.ma.masked_where(np.arange(dr['maxcount']) >= dr['count'][:,None], dr['indices'], copy=False)

    return data

def estimate_translation(data):
    d1, d2 = data['images']
    data['axis'] = [{'name': 'y'}, {'name': 'x'}]
    for a, a1, a2 in zip(data['axis'], d1['axis'], d2['axis']):
        a['dir'] = [{'dir': 1}, {'dir': -1}]
        for dr, dr1, dr2 in zip(a['dir'], a1['dir'], a2['dir']):
            counts = np.stack(np.broadcast_arrays(dr1['count'][:,None], dr2['count'][None,:]))
            mincounts = np.min(counts, axis=0)
            numdims = min(dr1['maxcount'], dr2['maxcount'])

            dr['side'] = [{'name': 'left', 'coeff': 1}, {'name': 'right', 'coeff': -1}]
            for side in dr['side']:
                if side['name'] == 'left':
                    countA = dr1['count'][:,None]
                    ptsA = dr1['indices'][:,None]
                    ptsB = dr2['indices'][None,:]
                else:
                    countA = dr2['count'][None,:]
                    ptsA = dr2['indices'][None,:]
                    ptsB = dr1['indices'][:,None]

                pts_shape = mincounts.shape + (numdims,)
                ptsA_shape = mincounts.shape + (ptsA.shape[-1],)

                roll = mincounts - countA
                ptsA = strided_indexing_roll(np.broadcast_to(ptsA, ptsA_shape), roll)
                
                side['pts'] = ptsB[...,:numdims] - ptsA[...,:numdims]

            # Each side represents a different option
            dr['pts'] = np.ma.stack([side['pts'] * side['coeff'] for side in dr['side']])

        # Combine rising and falling into one test
        a['pts'] = np.ma.concatenate([dr['pts'] for dr in a['dir']], axis=-1)
        numdiags = np.max(a['pts'].shape[:2])
        offsets = np.arange(-numdiags, numdiags)
        diags = np.moveaxis(diagonals(np.moveaxis(a['pts'],0,2), offsets), 2, 0)
        pts_offset = diags.reshape(list(diags.shape[:-2])+[-1])
        loss, dist = to_diag(pts_offset)
        numdims = pts_offset.shape[-1] - np.ma.count_masked(pts_offset, axis=-1)
        
        loss = loss / np.sqrt(numdims)
        a['dist'] = np.take_along_axis(dist, np.ma.argmin(loss, axis=0)[None,:], axis=0)[0]
        a['loss'] = np.min(loss, axis=0)

        if a['name'] == 'y':
            x, y = offsets, a['dist']
        else:
            x, y = a['dist'], offsets
            
        a['coords'] = np.stack([x, y], axis=-1)
        a['tree'] = KDTree(a['coords'])

    tree_vert, tree_horiz = (a['tree'] for a in data['axis'])
    coords_vert, coords_horiz = (a['coords'] for a in data['axis'])
    loss_vert, loss_horiz = (a['loss'] for a in data['axis'])
    matches = tree_horiz.query_ball_tree(tree_vert, 50, eps=0.1)
    close_matches = []
    min_loss = []
    for verts, pnt_horiz, err_horiz in zip(matches, coords_horiz, loss_horiz):
        if len(verts) == 0 or np.ma.count_masked(pnt_horiz) > 0:
            continue
        pnts_vert = coords_vert[verts]
        err_vert = loss_vert[verts]
        dist_sq = np.sum((pnts_vert - pnt_horiz) ** 2)
        loss = err_horiz * err_vert * dist_sq
        min_idx = np.argmin(loss)
        close_matches.append([pnt_horiz, pnts_vert[min_idx]]) 
        min_loss.append(loss[min_idx])
    min_idx = np.argmin(min_loss)
    close_match = close_matches[min_idx]
    translation = np.mean(close_match, axis=0)
    loss = min_loss[min_idx]

    return translation, loss

from sklearn.linear_model import RANSACRegressor
from sklearn.base import BaseEstimator

class DiagEstimator(BaseEstimator):
    def __init__(self, translation=0):
        self.translation = translation
    
    def fit(self, X, y):
        loss, self.translation = to_diag(y - X[:,0])

    def predict(self, X):
        return X[:,0] + self.translation

    def score(self, X, y):
        y_true = y
        y_pred = self.predict(X)
        u = ((y_true - y_pred) ** 2).sum()
        loss = u / np.sqrt(len(y_pred))
        score = -np.log(1 + loss)
        return score

ransac = RANSACRegressor(base_estimator=DiagEstimator(), min_samples=1)
def estimate_translation_guess(data, guess):
    try:
        d1, d2 = data['images']
        data['axis'] = [{'name': 'y'}, {'name': 'x'}]
        for a, a1, a2, offs, guess_ax in zip(data['axis'], d1['axis'], d2['axis'], guess, guess[::-1]):
            a['dir'] = [{'dir': 1}, {'dir': -1}]
            for dr, dr1, dr2 in zip(a['dir'], a1['dir'], a2['dir']):
                # Prepare offset ranges
                range1 = np.array([0, len(dr1['indices'])])
                range2 = range1 + offs
                ranges = np.stack([range1, range2])
                ranges[:,0] += max(0, -ranges[1,0])
                ranges[:,1] -= max(0, ranges[1,1]-len(dr2['indices']))
                
                start1, end1 = ranges[0]
                start2, end2 = ranges[1]
                
                if start1 == end1:
                    return guess, -np.inf # Too far away!

                counts = np.stack([dr1['count'][start1:end1], dr2['count'][start2:end2]])
                mincounts = np.min(counts, axis=0)
                numdims = np.min(np.max(counts, axis=-1))
                if numdims == 0:
                    return guess, -np.inf # No features found!

                pts1 = dr1['indices'][start1:end1]
                pts2 = dr2['indices'][start2:end2]

                def rollback(ptsX, countX):
                    ptsX_shape = mincounts.shape + (ptsX.shape[-1],)
                    roll = mincounts - countX
                    return strided_indexing_roll(np.broadcast_to(ptsX, ptsX_shape), roll)

                if guess_ax < 0:
                    # If we guess negative in this axis, then rollback pts1 to match (we might have unmatched entries at start of pts1)
                    pts1 = rollback(pts1, counts[0])
                else:
                    pts2 = rollback(pts2, counts[1])
                    
                dr['pts'] = pts2[...,:numdims] - pts1[...,:numdims]

            # Combine rising and falling into one test
            a['pts'] = np.ma.concatenate([dr['pts'] for dr in a['dir']], axis=-1)

            y = a['pts'][~a['pts'].mask].reshape(-1)
            X0 = np.zeros_like(y)[:,None]
            ransac.fit(X0, y)
            a['dist'] = ransac.predict(np.array([[0]]))[0]
            a['score'] = ransac.score(X0, y)

        translation = np.array([a['dist'] for a in data['axis']][::-1])
        score = np.sum([a['score'] for a in data['axis']])
        return translation, score
    except:
        # Computation error
        return guess, -np.inf

def estimate_translation_iterative(data, guess, reps, stop_dist=3):
    for _ in range(reps):
        solution, score = estimate_translation_guess(data, np.round(guess).astype(int))
        if np.linalg.norm(solution - guess) < stop_dist:
            break
        guess = solution
    return solution, score

def draw_stitched(img1, img2, translation):
    centers = np.array([translation, (0,0)])
    sizes = np.array([img1.shape, img2.shape])[:,1::-1]

    min_pts = np.round(centers - sizes/2).astype(int)
    max_pts = min_pts + sizes

    min_pt = np.min(min_pts, axis=0)
    max_pt = np.max(max_pts, axis=0)

    w,h = max_pt - min_pt
    img_combined = np.zeros((h, w, 3), dtype=np.uint8)
    for img,pt0,pt1 in zip([img1, img2[...,::-1]], min_pts, max_pts):
        mask = np.sum(img, axis=-1) > 0
        mask_broad = np.broadcast_to(mask[...,None], img.shape)
        x0, y0 = pt0 - min_pt
        x1, y1 = pt1 - min_pt
        view = img_combined[y0:y1,x0:x1]
        np.putmask(view, mask_broad, img)
    
    return img_combined


### Mapping ###

class Map(object):

    def __init__(self, resolution,
        init_cols=100, init_rows=100, 
        init_shift_x=0., init_shift_y=0.,
        val_range=10, val_clip=5):

        self.matrix = np.zeros((init_rows, init_cols), dtype=np.int8)

        # Height is negative because we flip vertically
        self.cell_dims = np.array([1.0, -1.0], dtype=float)
        self.resolution = resolution

        self.shift = np.array([init_shift_x, init_shift_y], dtype=float)
        self.range, self.clip = val_range, val_clip
        self.last_update_time = rospy.Time.now()

    def index(self, coords):
        "Return row,col indices from x,y coordinates."
        idx = ((coords - self.shift) // self.cell_dims).astype(int) # (x,y) => (j,i)
        return idx[...,::-1] # (j,i) => (i,j)

    _corner_tf = np.array([[0,1],[0,0],[1,1],[1,0]])
    def corners(self, idx):
        "Return list of x,y coordinates of the corners of the cells at index i,j"
         # (i,j) => (j,i) => (x,y)
        origin = idx[...,::-1] * self.cell_dims + self.shift
        corner_offsets = self.cell_dims * self._corner_tf

        return origin[...,None,:] +  corner_offsets # Expand to (..., 4, 2)

    def centers(self, idx):
        "Return list of x,y coordinates of the centers of the cells at index i,j"
         # (i,j) => (j,i) => (x,y)
        origin = idx[...,::-1] * self.cell_dims + self.shift
        center_offset = self.cell_dims * [0.5,0.5]

        return origin + center_offset

    def validate_entry(self, idx):
        "Check if the indices are within the matrix and expand if necessary, returning new indices."
        if not idx.size:
            return idx # No elements
        
        min_idx = np.min(idx, axis=-2)
        max_idx = np.max(idx, axis=-2)
        transform = np.zeros_like(self.matrix.shape)

        while (min_idx + transform)[0] < 0:
            # Expand up
            transform[0] += self.matrix.shape[0]
            g = np.zeros_like(self.matrix)
            self.matrix = np.concatenate([g, self.matrix], axis=0)
        while (max_idx + transform)[0] >= self.matrix.shape[0]:
            # Expand down
            g = np.zeros_like(self.matrix)
            self.matrix = np.concatenate([self.matrix, g], axis=0)
        while (min_idx + transform)[1] < 0:
            # Expand left
            transform[1] += self.matrix.shape[1]
            g = np.zeros_like(self.matrix)
            self.matrix = np.concatenate([g, self.matrix], axis=1)
        while (max_idx + transform)[1] >= self.matrix.shape[1]:
            # Expand right
            g = np.zeros_like(self.matrix)
            self.matrix = np.concatenate([self.matrix, g], axis=1)

        # Update shift (i,j) => (j,i) => (x,y)
        self.shift -= transform[::-1] * self.cell_dims

        # Update indices
        idx += transform

        return idx

    def __getitem__(self, key):
        idx = self.index(key) # (x,y) => (i,j)

        # Find invalid entries
        invalid = np.max((idx < 0) | (idx >= self.matrix.shape), axis=-1)

        # Zero out the invalid indices
        idx[invalid] = (0, 0)

        return np.where(invalid, self._default_val, self.matrix[idx[...,0], idx[...,1]])

    def __setitem__(self, key, value):
        idx = self.index(key) # (x,y) => (i,j)
        idx = self.validate_entry(idx)
        self.matrix[idx[...,0], idx[...,1]] = value

    def add(self, img, center):
        assert(img.dtype == bool)
        img_change = img.astype(np.int8) * 2 - 1
        img_size = np.array(img.shape[1::-1])

        topleft = center + (img_size / 2) * (-1, 1)
        topleft_idx = self.index(topleft)
        botright_idx = topleft_idx + img.shape[:2]
        start_idx, end_idx = self.validate_entry(np.array([topleft_idx, botright_idx]))

        map_range = self.matrix[start_idx[0]:end_idx[0], start_idx[1]:end_idx[1]]
        map_range += img_change
        map_range.clip(-self.range, self.range, out=map_range)

        self.last_update_time = rospy.Time.now()

        return start_idx, end_idx

    def extents(self):
        "Returns [[xmin, xmax], [ymin, ymax]]"
        start = self.shift
        end = self.matrix.shape[::-1] * self.cell_dims + self.shift
        corners = np.stack([start, end])
        botleft = np.min(corners, axis=0)
        topright = np.max(corners, axis=0)
        extent = np.stack([botleft, topright],axis=1)
        return extent

    def plot(self, grid=True, **kwargs):
        "Shows a pyplot of the grid"
        fig, ax = plt.subplots()
        ax.imshow(self.matrix, extent=list(self.extents().flat), **kwargs)
        ax.xaxis.set_minor_locator(plt.MultipleLocator(abs(self.cell_dims[0])))
        ax.yaxis.set_minor_locator(plt.MultipleLocator(abs(self.cell_dims[1])))
        if grid:
            ax.grid(True,which='both',linestyle='-',linewidth=1)

    def to_preview_image(self):
        "Converts matrix to BGR image for previewing"
        below = self.matrix.clip(-self.clip, 0) / -self.clip
        above = self.matrix.clip(0, self.clip) / self.clip

        blue = below + above
        red = above
        green = above

        return (np.stack([blue, green, red], axis=-1) * 255).astype(np.uint8)

    def to_occgrid(self):
        grid = OccupancyGrid()

        # Create header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        grid.header = header

        # Build metadata
        metadata = MapMetaData()
        metadata.map_load_time = self.last_update_time
        metadata.resolution = 1 / self.resolution
        metadata.height, metadata.width = self.matrix.shape

        # - Create origin point pose
        origin = Pose()
        origin.position.x, origin.position.y = self.shift / self.resolution
        metadata.origin = origin

        grid.info = metadata

        # Convert matrix data to occupancy grid
        empty = self.matrix < -self.clip
        filled = self.matrix > self.clip
        known = empty | filled

        grid.data = list(np.where(known, np.where(filled, 100, 0), -1).T.flat)
        return grid

    
def draw_corners(img, start_idx, end_idx, color=(0,0,0)):
    topleft = np.array((start_idx[0], start_idx[1]))
    topright = np.array((start_idx[0], end_idx[1]))
    botleft = np.array((end_idx[0], start_idx[1]))
    botright = np.array((end_idx[0], end_idx[1]))

    size = end_idx - start_idx
    lengths = size / 5

    cv2.line(img, to_cv(topleft), to_cv(topleft + lengths * (0,1)), color, thickness=2) 
    cv2.line(img, to_cv(topleft), to_cv(topleft + lengths * (1,0)), color, thickness=2)

    cv2.line(img, to_cv(topright), to_cv(topright + lengths * (0,-1)), color, thickness=2) 
    cv2.line(img, to_cv(topright), to_cv(topright + lengths * (1,0)), color, thickness=2)

    cv2.line(img, to_cv(botleft), to_cv(botleft + lengths * (0,1)), color, thickness=2) 
    cv2.line(img, to_cv(botleft), to_cv(botleft + lengths * (-1,0)), color, thickness=2)

    cv2.line(img, to_cv(botright), to_cv(botright + lengths * (0,-1)), color, thickness=2) 
    cv2.line(img, to_cv(botright), to_cv(botright + lengths * (-1,0)), color, thickness=2)

### Control Logic ###

# Parameters
CONTROL_RATE = 5 # Hz
SPEED_TAKEOFF = 0.5 # m/s
SPEED_FLY = 1.0 # m/s
FLIGHT_HEIGHT = 1.5 # m
TARGET_FLOW_DIST = 1.0 # m
SPEED_STABILITY = 0.1 # m/s
DIST_KEEPOUT_TARGET = 0.7 # m
DIST_KEEPOUT_UNSTABLE = 0.1 # m
MAX_MAPPING_BUFFER = 5 # imgs

# Parameters - Image stitching
STITCH_MAX_DIST = 100
"Maximum distance in pixels without a match before we need to start over"

STITCH_MAX_FLOW_ERR = 50
"Maximum distance in pixels between estimated flow and matched flow"

STITCH_MIN_SCORE = -30
"Minimum score to count a match"

MAP_SCALE = 4
"Scale down factor for the map"

class Driver(object):

    def __init__(self, control=True):
        self.running = False
        self.control = control
        self.state = 'take_off'
        self.img_msg = None
        self.undistort = lambda img: img
        self.camera_initialized = False
        self.bridge = CvBridge()
        self.filter_red = Filter(**RED_FILTER)
        self.filter_yellow = Filter(**YELLOW_FILTER)
        self.flow_pos = np.zeros(2)
        self.img_flow_pos = None
        self.last_img_flow_pos = None
        self.last_pos = None
        self.img_map = None
        self.offset = np.zeros(2)
        self.img_last = None
        self.imgs_collected = []
        self.shown = False
        self.map = Map(resolution=RESOLUTION/MAP_SCALE)

    def register(self, ns=''):
        self.get_telemetry = rospy.ServiceProxy(ns + 'get_telemetry', clover.srv.GetTelemetry)
        if self.control:
            self.navigate = rospy.ServiceProxy(ns + 'navigate', clover.srv.Navigate)
            self.navigate_global = rospy.ServiceProxy(ns + 'navigate_global', clover.srv.NavigateGlobal)
            self.set_position = rospy.ServiceProxy(ns + 'set_position', clover.srv.SetPosition)
            self.set_velocity = rospy.ServiceProxy(ns + 'set_velocity', clover.srv.SetVelocity)
            self.set_attitude = rospy.ServiceProxy(ns + 'set_attitude', clover.srv.SetAttitude)
            self.set_rates = rospy.ServiceProxy(ns + 'set_rates', clover.srv.SetRates)
            self.land = rospy.ServiceProxy(ns + 'land', Trigger)
        else:
            Retval = namedtuple('Retval', ['success', 'msg'])
            dummy_func = lambda **kwargs: Retval(success=True, msg="Dummy retval")
            self.navigate = dummy_func
            self.navigate_global = dummy_func
            self.set_position = dummy_func
            self.set_velocity = dummy_func
            self.set_attitude = dummy_func
            self.set_rates = dummy_func
            self.land = dummy_func

        self.image_sub = rospy.Subscriber(ns + '/main_camera/image_raw', 
                                          Image, self.image_callback, queue_size=1)

        self.caminfo_sub = rospy.Subscriber(ns + '/main_camera/camera_info', 
                                          CameraInfo, self.caminfo_callback)

        self.oflow_sub = rospy.Subscriber(ns + '/optical_flow/shift',
                                          Vector3Stamped, self.oflow_callback)
        self.map_pub = rospy.Publisher('/turtle/map', OccupancyGrid, queue_size=1)

        rospy.on_shutdown(self.shutdown)

    def main_loop(self):
        rate = rospy.Rate(CONTROL_RATE)

        self.running = True

        while not rospy.is_shutdown():
            if self.running:
                self.branch_state()
                self.build_map()

            rate.sleep()

    ### State Machine ###

    def branch_state(self):
        print("State:", self.state)
        if self.state == 'take_off':
            self.state_take_off()
        elif self.state == 'taking_off':
            self.state_taking_off()
        elif self.state == 'find_edge':
            self.state_find_edge()

    def state_take_off(self):
        telemetry = self.get_telemetry()
        retval = self.navigate(z=FLIGHT_HEIGHT - telemetry.z, speed=SPEED_TAKEOFF, frame_id='body', auto_arm=True)
        if retval.success:
            self.state = 'taking_off'
        else:
            print(retval.message)

    def state_taking_off(self):
        telemetry = self.get_telemetry()
        if np.abs(telemetry.z - FLIGHT_HEIGHT) < 0.1 and np.abs(telemetry.vz) < 0.1:
            self.state = 'find_edge'

    def state_find_edge(self):
        if self.img_msg is None or not self.camera_initialized:
            return
        img = self.bridge.imgmsg_to_cv2(self.img_msg,desired_encoding='bgr8')

        # Process image
        img = self.undistort(img)
        img_below, _ = project_below(img, self.rotation, self.translation, self.K, self.K_inv)

        # Filter image for boundary
        hsv = cv2.cvtColor(img_below, cv2.COLOR_BGR2HSV)
        filter = self.filter_red.apply(hsv)
        filter_img = (filter * 255).astype(np.uint8)
        edges = cv2.Canny(filter_img, 20, 240)

        edges_idx = np.moveaxis(np.indices(edges.shape),0,-1)[edges>127].reshape(-1,2)
        edges_xy = idx_to_xy(edges_idx)

        if edges_xy.shape[0] < 10:
            # No boundary found
            if np.linalg.norm(self.velocity) <= SPEED_STABILITY:
                # Not moving, just go forward
                self.set_velocity(vx=SPEED_FLY, frame_id='body')
            cv2.imshow("Camera", img)
            cv2.imshow("Below", img_below)
            cv2.waitKey(3)
            return

        # Draw edge points
        img_edges = np.zeros_like(img_below)
        img_edges[tuple(xy_to_idx(edges_xy).astype(int).clip((0,0),(HEIGHT-1,WIDTH-1)).T)] = (0, 0, 255)

        # Find closest point
        center_idx = np.array(img_below.shape[:2])/2
        center_xy = idx_to_xy(center_idx)

        closest_pt_xy = edges_xy[np.argmin(np.linalg.norm(edges_xy - center_xy, axis=-1))] 
        closest_pt_idx = xy_to_idx(closest_pt_xy)

        # Draw line to closest point
        cv2.line(img_below, to_cv(center_idx), to_cv(closest_pt_idx), (0,255,255), 2)
        cv2.line(img_edges, to_cv(center_idx), to_cv(closest_pt_idx), (0,255,255), 1)

        # Control drone
        vector = closest_pt_xy - center_xy
        dist = np.linalg.norm(vector) / RESOLUTION

        # If too close to edge, wait to stabilize
        if dist > DIST_KEEPOUT_UNSTABLE or np.linalg.norm(self.velocity) <= SPEED_STABILITY:
            dir = turn_left(vector)
            offset = vector / np.linalg.norm(vector) * (dist - DIST_KEEPOUT_TARGET)
            dir_phy = self.direct(dir, offset)

            # Draw control line
            control_phy = dir_phy + offset
            control_pt_xy = center_xy + control_phy * RESOLUTION
            control_pt_idx = xy_to_idx(control_pt_xy)

            cv2.line(img_below, to_cv(center_idx), to_cv(control_pt_idx), (0,255,0), 2)
            cv2.line(img_edges, to_cv(center_idx), to_cv(control_pt_idx), (0,255,0), 1)

        # Show imagery
        cv2.imshow("Camera", img)
        cv2.imshow("Below", img_below)
        cv2.imshow("Parsed", img_edges)
        cv2.waitKey(3)
        

    def stop(self):
        self.running = False
        self.land()

    ### Build mapping ###

    def build_map(self):
        if self.img_msg is None or not self.camera_initialized:
            return

        # Update flow
        #pos = self.img_flow_pos
        #if self.last_img_flow_pos is not None:
        #    flow = pos - self.last_img_flow_pos

        pos = self.translation[:2][::-1] * RESOLUTION
        if self.last_pos is not None:
            flow_below = pos - self.last_pos

        img = self.bridge.imgmsg_to_cv2(self.img_msg,desired_encoding='bgr8')

        # Process image
        img = self.undistort(img)
        img_below, H_inv = project_below(img, self.rotation, self.translation, self.K, self.K_inv)

        # Create view filter
        filter_view = np.sum(img_below, axis=-1) > 0

        # Create wall filter
        hsv_below = cv2.cvtColor(img_below, cv2.COLOR_BGR2HSV)
        filter_wall = self.filter_yellow.apply(hsv_below) > 0.5

        # Apply filters
        img_masked = np.ma.array(filter_wall, mask=~filter_view, copy=False)
        img_below[~filter_wall] = (0,0,0)
        img_masked_scaled = img_masked[::MAP_SCALE, ::MAP_SCALE]

        # If not initialized or too far away, start over
        if self.img_last is None or np.linalg.norm(flow_below) > STITCH_MAX_DIST:
            #self.last_img_flow_pos = pos
            self.last_pos = pos
            self.img_last = img_below
            self.img_masked_last = img_masked_scaled
            return

        # Project flow
        #center = np.array(img.shape[:2][::-1]) / 2
        #start = center - flow
        #start_below, center_below = apply_hom(H_inv, np.array([start, center]), inv=True)
        #flow_below = center_below - start_below

        # Attempt to stitch the image
        img1, img2 = self.img_masked_last.data, img_masked_scaled.data
        data = initialize_data(img1, img2)
        translation_origins_scaled, score = estimate_translation_iterative(data, flow_below, reps=3)
        translation_origins = translation_origins_scaled * MAP_SCALE

        # Convert origins offset to center offset
        center_cur = np.array(img_below.shape[:2][::-1]) / 2
        center_last = np.array(self.img_last.shape[:2][::-1]) / 2
        center_offset = center_cur - center_last
        translation = translation_origins + center_offset

        img_stitched = draw_stitched(self.img_last, img_below, translation)
        error = np.linalg.norm(translation - flow_below)
        offset = translation - flow_below

        if error < STITCH_MAX_FLOW_ERR and score > STITCH_MIN_SCORE:
            self.offset += offset
            refined_pos = pos + self.offset
            #self.flow_pos = self.flow_pos + offset
            #self.last_img_flow_pos = refined_pos
            self.last_pos = pos
            self.img_last = img_below
            self.img_masked_last = img_masked_scaled

            print('est pos:', pos)
            print('flow below:', flow_below)
            print('true below:', translation)
            print('true pos:', refined_pos)
            print('accrued offset:', self.offset)
            
            # Save the image!
            start_idx, end_idx = self.map.add(img_masked_scaled, refined_pos * (-1/MAP_SCALE,1/MAP_SCALE))

            # Preview the map
            img_preview = self.map.to_preview_image()
            draw_corners(img_preview, start_idx, end_idx, color=(0,0,255))
            cv2.imshow("Map", cv2.resize(img_preview, (512,512)))

            # Publish the map
            grid = self.map.to_occgrid()
            self.map_pub.publish(grid)
        
        if not self.shown:
            cv2.imshow("Stitched", img_stitched)
            self.shown = True
        cv2.addText(img_stitched, f"score: {score:.2f}", (100, 100), "Calibri", pointSize=12, color=(255,255,255))
        cv2.addText(img_stitched, f"error: {error:.2f}", (100, 120), "Calibri", pointSize=12, color=(255,255,255))
        cv2.addText(img_stitched, f"pos: ({self.translation[0]:.1f},{self.translation[1]:.1f})", (100, 140), "Calibri", pointSize=12, color=(255,255,255))
        cv2.imshow("Stitched", img_stitched)
        cv2.waitKey(3)


    ### Control the drone's flight path ###

    def direct(self, direction, offset=(0,0)):
        dir_unit = direction / np.linalg.norm(direction)
        vector = TARGET_FLOW_DIST * dir_unit
        dx, dy = to_clover(vector + offset)
        self.navigate(x=dx, y=dy, speed=SPEED_FLY, frame_id='body')
        return vector

    ### Receive and parse callbacks from the drone ###

    def image_callback(self, msg):
        telemetry = self.get_telemetry()
        self.rotation, self.translation, self.velocity = parse_telemetry(telemetry)
        self.img_flow_pos = self.flow_pos
        self.img_msg = msg

    def oflow_callback(self, msg):
        #time = rospy.Time(msg.header.stamp)
        #delta_t = time - self.last_flow_time
        delta_x = np.array([msg.vector.x, msg.vector.y])
        if np.linalg.norm(delta_x) > 4.0: # outlier, bad reading
            return # skip
        self.flow_pos = self.flow_pos + delta_x
        #self.last_flow_time = time

    def caminfo_callback(self, msg):
        # Only need to initialize once
        if self.camera_initialized:
            return

        print("-- Initializing Camera --")

        # Image dimensions
        image_size = (msg.width, msg.height)
        print("Height:", msg.height)
        print("Width:", msg.width)

        # Camera Intrinsic Matrix
        camera_matrix = np.array(msg.K).reshape((3,3))
        self.K = camera_matrix
        self.K_inv = np.linalg.inv(camera_matrix)
        fovx, fovy, focalLength, principalPoint, aspectRatio = cv2.calibrationMatrixValues(camera_matrix, image_size, 0, 0)
        print("FOV_X:", fovx)
        print("FOV_Y:", fovy)
        print("Focal Length:", focalLength)
        print("Principal Point:", principalPoint)
        print("Aspect Ratio:", aspectRatio)

        # Image distortion
        if np.count_nonzero(msg.D) > 0:
            if msg.distortion_model == 'plumb_bob':
                self.undistort = undistort_plumb_bob(camera_matrix, msg.D, image_size)
                print("Loaded 'plumb-bob' distortion model.")
            else:
                print(f"Unsupported distortion model '{msg.distortion_model}'.")

        # Camera parameter loading complete
        print("-- END --")
        self.camera_initialized = True
        self.caminfo_sub.unregister()
        

    def shutdown(self):
		# Gracefully shut down all of our windows/processes
        
        self.stop()
        print("Shutting down the drone_control node...")

        try:
            self.map.plot(grid=False)

        except:
            e = sys.exc_info()[1]
            print("There was a problem shutting down.")
            print(e)

        return

def parse_args():
    kwargs = {}

    #if rospy.has_param('~resolution'):
    #    kwargs['resolution'] = rospy.get_param('~resolution')
    #else:
    #    rospy.logwarn(f"PARAMS: Resolution not provided; using {GRID_RESOLUTION_DEFAULT:.1f}")

    if rospy.has_param('~control'):
        kwargs['control'] = rospy.get_param('~control')
        if not kwargs['control']:
            rospy.loginfo("PARAMS: Drone control disabled.")

    return kwargs

def main():
    rospy.init_node('drone_control', anonymous=False)
    kwargs = parse_args()
    driver = Driver(**kwargs)
    driver.register()
    driver.main_loop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("drone_control node terminated")