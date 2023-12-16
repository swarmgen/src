x <- c(-1, -1.3, -1.6, -1.4)
y <- c(0.9, 1.2, 0.8, 0.4)



rand_coord = "/mnt/storage1/Github/11_SwarmSlicing/Project_data_analysis/QE/M1/input/gen_rand_coord.csv"

data_raw <- read.csv(file = rand_coord, header = TRUE, fileEncoding="UTF-8-BOM", as.is = 1, sep = ",")


plot(data_raw$l_x, data_raw$l_y, xlim = c(-2, 0), ylim = c(-0.5, 2.0), col = "red", pch = 19)
points(data_raw$f1_x, data_raw$f1_y, col = "blue", pch = 19)

points(data_raw$f2_x, data_raw$f2_y, col = "orange", pch = 19)

points(data_raw$f3_x, data_raw$f3_y, col = "green", pch = 19)
