import matplotlib.pyplot as plt

f = "/root/catkin_ws/src/project3/src/learning_graphs/learning_graph.2.28.23.1834.txt"
file = open(f, "r")

#array = file.read()
array = [[0, 5], [0, 0], [0, 3], [0, 2], [0, 3], [0, 7], [0, 2], [0, 3], [0, 2], [0, 7], [0, 7], [0, 1], [0, 9], [0, 18], [0, 5], [11, 11], [0, 4], [0, 4], [0, 3], [0, 4], [5, 9], [4, 6], [0, 0], [6, 8], [8, 8], [0, 18], [10, 45], [11, 15], [12, 12], [0, 5], [13, 13], [2, 34], [0, 9], [0, 6], [0, 6], [0, 10], [4, 8], [10, 10], [23, 32], [33, 48], [16, 22], [12, 17], [23, 23], [21, 50], [5, 48], [27, 38], [16, 16], [10, 10], [11, 11], [4, 8], [20, 20], [4, 12], [14, 22], [20, 23], [4, 4], [2, 8], [25, 25], [26, 29], [4, 6], [0, 5], [11, 11], [16, 16], [28, 35], [4, 7], [25, 41], [37, 39], [21, 43], [9, 9], [19, 74], [23, 34], [28, 43], [0, 13], [0, 13], [0, 9], [0, 12], [12, 69], [14, 38], [15, 30], [17, 18], [14, 64], [49, 114], [39, 96], [41, 61], [0, 4], [49, 123], [55, 64], [19, 19], [12, 21], [6, 12], [26, 33], [34, 44], [16, 16], [5, 12], [0, 11], [7, 24], [16, 25], [39, 39], [0, 31], [0, 12], [35, 55], [0, 25], [0, 34], [0, 47], [0, 62], [33, 33], [4, 13], [0, 52], [0, 19], [0, 12], [0, 13], [0, 11], [0, 24], [0, 8], [0, 9], [0, 66], [0, 10], [11, 32], [0, 97], [0, 43], [0, 15], [0, 11], [0, 10], [0, 9], [0, 13], [0, 9], [6, 26], [2, 13], [1, 8], [0, 16], [0, 11], [13, 25], [29, 29], [2, 69], [29, 129], [0, 53], [0, 31], [0, 19], [0, 115], [20, 46], [86, 103], [25, 25], [41, 41], [0, 2], [0, 3], [0, 9], [0, 14], [0, 11], [0, 10], [0, 10], [0, 10], [0, 2], [35, 39], [0, 43], [0, 12], [0, 53], [0, 28], [13, 60], [0, 107], [0, 114], [0, 35], [0, 17], [0, 86], [0, 56], [0, 14], [0, 13], [46, 53], [22, 60], [0, 0], [0, 23], [0, 23], [0, 10], [0, 35], [0, 61], [0, 141], [0, 28], [2, 130], [0, 15], [0, 12], [0, 63], [0, 4], [0, 27], [0, 27], [0, 10], [25, 30], [25, 25], [0, 10], [0, 60], [70, 231], [7, 51], [0, 14], [49, 84], [0, 14], [15, 21], [0, 14], [0, 11], [0, 33], [35, 35], [61, 69], [24, 52], [0, 24], [0, 14], [36, 122], [0, 57], [0, 0], [0, 169], [0, 190], [45, 76], [15, 79], [16, 37], [69, 79], [147, 263], [136, 286], [103, 248], [0, 28], [0, 20], [0, 19], [0, 186], [24, 28], [0, 11], [1, 73], [0, 55], [17, 87], [0, 33], [14, 18], [20, 55], [19, 59], [0, 30], [12, 12], [21, 83], [0, 84], [0, 39], [0, 17], [0, 31], [0, 55], [17, 48], [18, 25], [17, 46], [18, 42], [14, 45], [16, 41], [0, 15], [14, 41], [12, 65], [11, 17], [51, 61], [0, 150], [0, 180], [10, 141], [0, 169], [0, 153], [0, 1], [0, 0], [0, 23], [0, 23], [0, 6], [0, 29], [0, 7], [0, 26], [0, 1], [11, 116], [0, 230], [0, 219], [0, 224], [96, 201], [174, 313], [17, 24], [56, 64], [0, 38], [0, 8], [0, 11], [47, 82], [1, 20], [125, 296], [87, 283], [0, 30], [15, 99], [0, 28], [0, 17], [0, 11], [30, 34], [58, 58], [96, 96], [67, 67], [20, 20], [24, 24], [60, 60], [79, 79], [1, 9], [3, 10], [22, 22], [32, 56], [0, 20], [14, 39], [48, 67], [46, 68], [0, 25], [0, 17], [0, 15], [183, 332], [0, 0], [165, 339], [118, 275], [45, 45], [0, 30], [0, 11], [0, 10], [49, 65], [0, 13], [0, 14], [0, 14], [56, 65], [0, 8], [0, 11], [0, 12], [0, 12], [0, 34], [0, 12], [0, 13], [38, 76], [22, 22], [40, 128], [12, 25], [44, 65], [44, 93], [0, 11], [56, 106], [36, 51], [17, 27], [0, 17], [0, 267], [0, 251], [0, 120], [28, 28], [62, 70], [19, 55], [15, 17], [0, 18], [0, 53], [64, 71], [22, 29], [0, 45], [21, 58], [0, 68], [4, 60], [14, 38], [10, 84], [0, 192], [0, 217], [0, 199], [0, 119], [4, 65], [29, 37], [15, 15], [94, 102], [37, 63], [32, 89], [12, 43], [17, 39], [18, 77], [0, 7], [0, 14], [14, 17], [0, 106], [0, 28], [0, 13], [0, 103], [0, 2], [0, 25], [0, 220], [0, 283], [0, 260], [0, 68], [24, 67], [0, 33], [0, 9], [0, 253], [17, 30], [35, 35], [77, 83], [23, 27], [26, 26], [0, 14], [3, 17], [0, 15], [20, 26], [0, 26], [0, 14], [0, 31], [0, 11], [0, 12], [53, 89], [0, 28], [0, 13], [4, 13], [75, 83], [0, 0], [0, 28], [0, 23], [0, 13], [0, 15], [0, 9], [0, 11], [47, 80], [18, 24], [8, 17], [4, 14], [62, 65], [12, 13], [9, 15], [0, 10], [0, 13], [0, 32], [0, 10], [0, 14], [64, 72], [11, 72], [4, 42], [70, 143], [63, 79], [0, 0], [0, 23], [4, 23], [0, 12], [18, 36], [31, 66], [69, 92], [45, 81], [0, 15], [0, 14], [0, 13], [55, 70], [0, 2], [0, 14], [0, 13], [1, 12], [21, 26], [1, 10], [0, 10], [0, 57], [0, 29], [9, 17], [28, 49], [0, 323], [0, 39], [0, 26], [0, 10], [0, 13], [4, 20], [6, 43], [20, 68], [52, 87], [0, 26], [25, 25], [23, 62], [9, 36], [0, 0], [0, 30], [0, 25], [1, 12], [14, 20], [0, 10], [69, 81], [60, 73], [7, 17], [64, 73], [56, 66], [80, 86], [0, 7], [19, 23], [73, 347], [1, 244], [0, 19], [41, 55], [19, 59], [0, 42], [0, 128], [9, 18], [71, 211], [21, 29], [5, 12], [21, 24], [22, 27], [28, 28], [0, 36], [0, 10], [0, 12], [0, 17], [0, 13], [0, 11], [0, 13], [0, 16], [0, 10], [16, 18], [20, 26], [1, 13], [12, 18], [1, 11], [0, 10], [0, 13], [0, 15], [0, 14], [0, 15], [21, 29], [0, 10], [0, 14], [101, 159], [24, 29], [4, 65], [17, 59], [255, 526], [194, 410], [18, 408], [16, 84], [0, 15], [0, 57], [0, 17], [0, 27], [0, 88], [46, 236], [17, 24], [55, 69], [21, 55], [0, 41], [131, 408], [0, 19], [4, 16], [0, 17], [10, 19], [17, 22], [15, 18], [0, 12], [0, 14], [0, 9], [0, 11], [21, 65], [42, 100], [3, 37], [56, 107], [54, 70], [15, 24], [235, 543], [40, 102], [59, 65], [0, 33], [0, 8], [0, 12], [48, 60], [0, 15], [7, 16], [0, 13], [56, 71], [0, 14], [0, 15], [18, 24], [1, 11], [13, 18], [0, 9], [1, 9], [58, 65], [17, 21], [0, 14], [3, 14], [65, 74], [11, 15], [0, 20], [52, 256], [30, 54], [12, 20], [18, 38], [72, 82], [21, 56], [17, 22], [0, 19], [2, 13], [0, 44], [0, 6], [0, 13], [0, 13], [0, 11], [0, 15], [0, 10], [74, 89], [74, 74], [25, 25], [33, 33], [47, 57], [57, 62], [18, 39], [0, 15], [0, 186], [0, 203], [0, 35], [0, 20], [0, 9], [0, 62], [19, 24], [7, 16], [0, 14], [0, 59], [0, 6], [17, 22], [19, 26], [44, 54], [0, 36], [0, 10], [0, 12], [0, 62], [0, 44], [0, 14], [0, 14], [17, 50], [11, 32], [0, 16], [0, 16], [0, 11], [9, 15], [1, 10], [0, 11], [62, 76], [3, 23], [0, 6], [43, 61], [31, 62], [0, 2], [0, 28], [0, 29], [57, 65], [13, 28], [27, 65], [40, 40], [0, 29], [14, 20], [0, 15], [8, 62], [0, 30], [29, 189], [0, 29], [0, 27], [0, 13], [0, 13], [0, 10], [0, 11], [49, 63], [26, 26], [34, 34], [67, 67], [66, 73], [5, 15], [5, 16], [15, 20], [1, 10], [0, 13], [30, 63], [72, 83], [60, 69], [14, 22], [37, 37], [3, 14], [73, 80], [0, 0], [0, 26], [0, 28], [13, 255], [0, 71], [1, 46], [2, 405], [0, 46], [19, 86], [19, 47], [33, 49], [154, 412], [0, 0], [0, 14], [19, 76], [28, 43], [8, 14], [2, 11], [1, 14], [48, 60], [15, 39], [22, 44], [17, 36], [72, 81], [24, 24], [22, 22], [11, 13], [1, 14], [15, 20], [1, 11], [44, 68], [179, 302], [232, 498], [60, 271], [19, 43], [8, 87], [0, 13], [0, 26], [0, 25], [1, 12], [0, 26], [0, 15], [1, 129], [175, 362], [0, 93], [1, 291], [19, 43], [11, 47], [19, 49], [13, 19], [0, 14], [0, 11], [2, 73], [56, 70], [6, 151]]
#array = [[0, 0], [0, 0],  [0, 0], [0, 0], [0, 0], [0, 0], [4, 8], [0, 1], [0, 0], [0, 0], [1, 1], [0, 0], [1, 1], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 5], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [0, 1], [0, 0], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 2], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [1, 2], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 2], [0, 0], [0, 0], [1, 3], [0, 0], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [2, 2], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 3], [0, 0], [1, 4], [2, 4], [0, 0], [0, 0], [0, 0], [1, 7], [0, 0], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [2, 7], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [2, 3], [0, 0], [0, 0], [0, 0], [1, 2], [2, 2], [6, 21], [0, 1], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 2], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 1], [0, 0], [0, 4], [2, 5], [0, 0], [0, 0], [1, 3], [1, 3], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 3], [0, 0], [1, 1], [1, 4], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [2, 2], [1, 1], [1, 2], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [3, 8], [0, 0], [0, 1], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [2, 2], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [2, 4], [0, 1], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 3], [0, 0], [0, 0], [0, 0], [0, 2], [1, 3], [0, 1], [1, 5], [2, 8], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 2], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [3, 3], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [3, 5], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 2], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 6], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 2], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 3], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 4], [0, 0], [1, 4], [1, 4], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 4], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 3], [0, 0], [2, 2], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 3], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [1, 6], [1, 1], [0, 0], [1, 4], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 2], [0, 1], [0, 0], [5, 13], [0, 0], [3, 4], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [2, 2], [1, 2], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [2, 2], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [4, 4], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [6, 13], [1, 10], [0, 0], [0, 0], [0, 0], [0, 0], [1, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
#array = [[1, 5], [0, 6], [1, 8], [1, 4], [0, 4], [3, 10], [5, 6], [3, 7], [3, 3], [8, 9], [24, 29], [2, 6], [10, 13], [11, 13], [7, 7], [5, 7], [2, 6], [0, 5], [23, 28], [6, 9], [5, 12], [6, 14], [1, 12], [0, 4], [3, 8], [1, 19], [20, 25], [0, 9], [6, 7], [0, 6], [9, 15], [18, 27], [4, 13], [7, 7], [1, 15], [0, 7], [1, 19], [0, 14], [6, 12], [0, 13], [7, 28], [8, 33], [30, 38], [27, 29], [17, 18], [2, 7], [28, 37], [6, 9], [15, 16], [13, 17], [2, 6], [5, 10], [0, 7], [0, 26], [6, 17], [7, 20], [0, 3], [0, 9], [3, 31], [1, 9], [11, 23], [8, 18], [24, 46], [9, 16], [10, 22], [0, 15], [10, 28], [7, 21], [5, 5], [17, 27], [24, 39], [8, 10], [7, 18], [8, 30], [6, 14], [29, 40], [25, 37], [22, 32], [15, 31], [13, 32], [39, 63], [63, 109], [35, 56], [7, 16], [44, 94], [3, 40], [10, 35], [5, 17], [5, 46], [7, 48], [45, 67], [3, 21], [7, 17], [11, 11], [48, 76], [4, 18], [3, 22], [8, 42], [0, 12], [17, 43], [14, 25], [41, 56], [72, 112], [40, 69], [5, 11], [0, 14], [47, 59], [11, 22], [0, 27], [2, 23], [1, 12], [19, 19], [0, 13], [0, 13], [74, 86], [6, 20], [18, 43], [42, 91], [50, 59], [6, 19], [0, 35], [5, 32], [16, 16], [0, 13], [0, 12], [12, 28], [4, 19], [16, 30], [0, 13], [1, 16], [1, 1], [8, 16], [26, 71], [22, 56], [0, 13], [4, 48], [14, 46], [3, 31], [43, 61], [16, 22], [4, 21], [0, 18], [16, 51], [8, 22], [5, 68], [5, 29], [5, 17], [4, 13], [1, 7], [6, 18], [4, 55], [5, 28], [32, 118], [21, 51], [66, 110], [18, 49], [39, 90], [71, 131], [100, 127], [23, 73], [22, 45], [62, 80], [44, 69], [4, 10], [32, 32], [24, 27], [67, 67], [11, 27], [17, 25], [16, 16], [2, 18], [39, 57], [45, 46], [22, 22], [1, 1], [74, 74], [5, 16], [4, 13], [71, 71], [15, 19], [14, 21], [2, 20], [0, 15], [4, 16], [0, 15], [2, 13], [58, 75], [108, 121], [26, 40], [0, 16], [26, 38], [9, 23], [3, 21], [2, 29], [0, 18], [0, 18], [1, 16], [10, 30], [61, 70], [21, 22], [12, 15], [46, 134], [59, 74], [0, 2], [50, 51], [40, 40], [18, 47], [39, 60], [4, 23], [13, 34], [110, 122], [111, 112], [106, 117], [8, 26], [3, 23], [0, 1], [36, 50], [12, 25], [17, 17], [45, 55], [38, 53], [15, 33], [1, 1], [25, 30], [15, 32], [11, 32], [1, 1], [4, 16], [65, 83], [71, 88], [32, 48], [29, 33], [33, 37], [45, 60], [56, 73], [6, 25], [23, 44], [28, 52], [70, 91], [31, 45], [13, 34], [35, 93], [69, 120], [0, 18], [21, 50], [152, 231], [163, 270], [105, 247], [145, 259], [130, 149], [9, 63], [7, 19], [11, 41], [18, 34], [4, 42], [13, 36], [8, 33], [3, 18], [6, 33], [70, 93], [208, 223], [214, 233], [219, 232], [87, 95], [126, 138], [8, 27], [3, 16], [21, 23], [13, 17], [6, 16], [42, 43], [34, 53], [149, 167], [93, 107], [1, 1], [50, 94], [23, 23], [13, 38], [0, 18], [0, 10], [0, 47], [2, 22], [35, 58], [6, 33], [4, 12], [0, 27], [30, 60], [4, 23], [3, 46], [7, 16], [9, 29], [6, 29], [7, 36], [19, 42], [38, 56], [5, 20], [8, 24], [20, 20], [150, 167], [0, 0], [147, 161], [121, 133], [13, 19], [20, 21], [21, 21], [17, 19], [50, 50], [19, 43], [15, 23], [5, 19], [33, 73], [8, 23], [7, 40], [12, 50], [0, 13], [0, 15], [0, 15], [0, 17], [38, 48], [5, 32], [17, 99], [1, 58], [43, 60], [25, 41], [15, 32], [43, 65], [13, 37], [22, 25], [26, 61], [233, 247], [258, 278], [87, 96], [0, 19], [0, 17], [59, 75], [4, 14], [1, 36], [11, 76], [0, 16], [0, 18], [36, 65], [46, 69], [75, 112], [68, 108], [38, 60], [90, 153], [195, 300], [163, 299], [176, 321], [31, 72], [10, 10], [1, 1], [21, 22], [1, 1], [54, 67], [45, 54], [28, 53], [33, 51], [83, 107], [5, 24], [16, 33], [17, 34], [103, 180], [25, 63], [0, 42], [19, 147], [7, 61], [11, 52], [233, 261], [268, 281], [271, 290], [42, 128], [21, 47], [6, 20], [8, 20], [65, 89], [12, 24], [0, 0], [51, 55], [4, 5], [21, 21], [20, 22], [14, 23], [21, 29], [13, 25], [11, 21], [6, 22], [1, 2], [0, 18], [1, 18], [45, 47], [18, 18], [10, 24], [7, 20], [33, 49], [10, 29], [11, 28], [8, 27], [0, 18], [0, 16], [5, 16], [0, 16], [30, 42], [4, 30], [0, 17], [0, 17], [27, 66], [5, 18], [1, 43], [8, 48], [22, 22], [13, 18], [7, 18], [0, 16], [35, 49], [56, 94], [27, 46], [34, 101], [50, 50], [17, 42], [25, 28], [4, 21], [0, 19], [0, 19], [28, 44], [1, 16], [32, 49], [3, 19], [21, 22], [11, 20], [32, 43], [17, 24], [11, 22], [4, 22], [2, 18], [8, 24], [5, 16], [0, 18], [60, 81], [7, 19], [5, 19], [14, 30], [307, 323], [19, 36], [21, 43], [8, 46], [19, 55], [9, 59], [40, 67], [37, 48], [35, 47], [7, 18], [18, 18], [34, 52], [8, 8], [0, 0], [17, 17], [20, 24], [9, 19], [0, 17], [0, 21], [2, 22], [35, 58], [10, 35], [0, 18], [0, 12], [35, 52], [21, 22], [25, 26], [249, 249], [237, 240], [32, 32], [22, 35], [63, 63], [76, 87], [107, 121], [0, 15], [59, 134], [2, 3], [1, 17], [13, 23], [26, 27], [22, 22], [15, 20], [7, 23], [6, 15], [5, 24], [8, 18], [5, 23], [0, 15], [2, 26], [0, 9], [11, 29], [5, 22], [0, 14], [0, 26], [0, 14], [0, 12], [6, 25], [4, 19], [3, 24], [0, 3], [2, 3], [0, 17], [9, 34], [47, 136], [6, 37], [29, 62], [34, 53], [221, 222], [190, 215], [334, 334], [41, 58], [6, 23], [71, 87], [5, 15], [10, 19], [64, 86], [148, 210], [15, 31], [18, 36], [43, 58], [71, 90], [178, 196], [0, 17], [0, 18], [6, 29], [4, 25], [7, 29], [10, 32], [0, 19], [2, 21], [9, 19], [1, 19], [43, 61], [40, 73], [22, 55], [41, 100], [43, 62], [15, 37], [245, 246], [48, 66], [1, 20], [5, 17], [5, 14], [0, 12], [30, 46], [3, 24], [0, 16], [0, 18], [21, 38], [10, 32], [5, 24], [5, 25], [6, 17], [6, 22], [7, 21], [6, 17], [40, 51], [6, 32], [0, 22], [0, 15], [34, 59], [5, 20], [12, 33], [169, 237], [8, 33], [13, 32], [8, 54], [11, 31], [62, 76], [5, 23], [0, 16], [0, 11], [58, 82], [4, 21], [11, 29], [12, 32], [0, 13], [0, 15], [0, 16], [2, 22], [33, 57], [4, 6], [0, 17], [9, 29], [43, 61], [19, 37], [14, 31], [42, 45], [185, 264], [34, 65], [20, 32], [5, 20], [62, 69], [13, 29], [0, 11], [0, 15], [60, 74], [0, 14], [25, 31], [17, 26], [9, 26], [2, 4], [5, 18], [7, 24], [70, 70], [45, 58], [17, 22], [0, 21], [57, 94], [34, 59], [18, 36], [21, 39], [0, 15], [4, 16], [12, 12], [18, 21], [48, 99], [15, 83], [2, 32], [4, 78], [23, 43], [2, 20], [2, 21], [4, 19], [0, 18], [0, 31], [23, 44], [6, 15], [1, 2], [14, 27], [6, 23], [13, 13], [2, 2], [48, 48], [22, 22], [18, 21], [0, 14], [0, 16], [1, 7], [7, 18], [31, 40], [3, 25], [0, 12], [0, 23], [39, 56], [8, 22], [8, 33], [3, 26], [0, 17], [0, 17], [21, 41], [0, 16], [33, 55], [4, 28], [2, 23], [0, 18], [32, 49], [0, 1], [8, 16], [4, 21], [206, 224], [45, 68], [35, 59], [350, 369], [69, 88], [59, 85], [26, 48], [13, 40], [223, 256], [1, 1], [25, 34], [45, 92], [9, 39], [6, 16], [0, 16], [0, 22], [31, 50], [10, 58], [21, 43], [15, 50], [32, 50], [8, 22], [0, 22], [7, 41], [6, 22], [21, 21], [20, 21], [36, 45], [115, 221], [232, 284], [202, 266], [48, 59], [107, 112], [15, 17], [11, 22], [3, 20], [0, 13], [8, 26], [19, 77], [38, 45], [161, 227], [70, 85], [234, 254], [28, 44], [69, 90], [27, 50], [11, 28], [3, 20], [0, 22], [44, 69], [20, 37], [103, 138]]

values = []
for i in range(len(array)):
	if array[i][1] == 0 or array[i][1] < 20:
		values += [0]
	else:
		values += [array[i][0]/array[i][1]]

plt.plot(values)
plt.savefig("straight_learning.png")