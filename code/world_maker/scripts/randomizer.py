#!/usr/bin/env python3

import openpyxl
import random

wb = openpyxl.load_workbook('custom_world_coords.xlsx')
sheet1 = wb["Sheet1"]
i = 83
j = 83

while i < 170:

	x = str(i)
	index = 'a'+ x
	sheet1[index] = random.randrange(0,20,2)
	i = i + 1

while i < 170:

	x = str(i)
	index = 'b'+ x
	sheet1[index] = random.randrange(0,20,2)
	j = j + 1	
	
print("Workbook has been randomized")
wb.save('custom_world_coords.xlsx')
