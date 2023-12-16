##############
# Initializing
# tips: ctrl + shft + c: comment selected lines
# refer for plotting: https://www.statmethods.net/advgraphs/parameters.html
##############

rm(list = ls())
library(plyr)
library(readr)
library(scales)
library(conicfit)
library(geometry)
library(retistruct)
library(areaplot)
require("graphics")

machine <- "Mac"

if(machine == "Mac"){
  base_directory = paste("/Users/XX/Research/Project_data_analysis/",sep="" )
}else if(machine == "Linux"){
  base_directory = paste("/mnt/storage1/Github/11_SwarmSlicing/Project_data_analysis/",sep="" )
}else{
  base_directory = paste("/mnt/storage1/Github/11_SwarmSlicing/Project_data_analysis/",sep="" )
}

source(paste(base_directory, "lib/lib_func.R", sep = ""))
source(paste(base_directory, "lib/lib_handling_dcc.R", sep = ""))
source(paste(base_directory, "lib/lib_piecewise.R", sep = ""))

# directory for input 
input_directory = paste(base_directory,"input/piecewise_linear/",sep="" )
datalist <- list.files(path = input_directory)
datalist

i <- 2
filename <- datalist[i]
filename
# directory for plot-output
output_directory_plot = paste(base_directory,"output/plot/",filename,"/",sep="" )
dir.create(output_directory_plot)

##############
# Configurations for general setting
##############
set.seed(1)
param.is_prunning <- TRUE
# param.mode.savePlot = TRUE # Usually TRUE as this program is for getting plots.
# param.isCrash = FALSE # If there is crash case in random seed so you need to avoid it, flip this to TRUE.

DEBUG = FALSE

##############
# Parameters for getting data
##############
# start_tick = 1
# end_tick = 9999

# param.threshold = -999

# param.mode_set = c("max")#, "min", "avg")
# param.target_drone_set = c("d1")#, "d2", "d3", "swarm")

# rand_seed_to = 1 # <====== Usually manupulate this
# rand_seed_from = 1 # <====== Usually manupulate this
# rand_seed_start_set = c(rand_seed_from:rand_seed_to)






cat("[log] Current file is [", filename, "].\n")

csvname = paste(input_directory,filename,sep="" )
data_raw <- read.csv(file = csvname, header = TRUE, fileEncoding="UTF-8-BOM", as.is = 1, sep = ",")


#install.packages("segmented")
library(segmented)
library(ggplot2)
#install.packages("ggplot2")

# little preprocessing
data_raw2 <- data_raw
for (row_idx in c(1:nrow(data_raw))){
  for (col_idx in c(4:11)){
    data_raw2[row_idx,col_idx] <- data_raw[row_idx,col_idx] / sum(data_raw[row_idx,4:11])
    if(col_idx >= 5){
      data_raw2[row_idx,col_idx] <- data_raw2[row_idx, (col_idx - 1)] + data_raw2[row_idx, col_idx]
    }
  }
}



# set.seed(12)
# xx <- data_raw2$t
# yy <- data_raw2$delta_des
# 
# dati <- data.frame(x = xx, y = yy)
# out.lm <- lm(y ~ x, data = dati)
# o <- segmented(out.lm, segZ = ~x, npsi = 6, control = seg.control(display = FALSE))
# dat2 <- data.frame(x = xx, y = broken.line(o)$fit)

dat1 <- data.frame(x = data_raw2$t, y = data_raw2$delta_des)
dat1_1 <- func.piecewise(dat1, 6)

dat2 <- data.frame(x = data_raw2$t, y = data_raw2$delta_l)
dat2_1 <- func.piecewise(dat2, 10)

dat3 <- data.frame(x = data_raw2$t, y = data_raw2$delta_f2)
dat3_1 <- func.piecewise(dat3, 6)

dat4 <- data.frame(x = data_raw2$t, y = data_raw2$delta_f3)
dat4_1 <- func.piecewise(dat4, 6)

# set.seed(12)
xx <- data_raw2$t
yy <- data_raw2$delta_f3
# 
dati <- data.frame(x = xx, y = yy)
out.lm <- lm(y ~ x, data = dati)
out.lm
o <- segmented(out.lm, segZ = ~x, npsi = 20, control = seg.control(display = FALSE))
dat2 <- data.frame(x = xx, y = broken.line(o)$fit)


dat5 <- data.frame(x = data_raw2$t, y = data_raw2$delta_obs..1.)
dat5_1 <- func.piecewise(dat5, 6)

dat6 <- data.frame(x = data_raw2$t, y = data_raw2$delta_obs..2.)
dat6_1 <- func.piecewise(dat6, 6)

dat7 <- data.frame(x = data_raw2$t, y = data_raw2$delta_obs..3.)
dat7_1 <- func.piecewise(dat7, 6)

dat8 <- data.frame(x = data_raw2$t, y = data_raw2$delta_wall)
dat8_1 <- func.piecewise(dat8, 6)


ggplot(dat2, aes(x = x, y = y)) +
  # geom_area( fill="#69b3a2", alpha=0.4) +
  geom_line() +
  # geom_point() +
  # geom_line(data = dat1_1, color = 'blue') + 
  geom_line(data = dat2_1, color = 'red') +
  geom_line(data = dat3, color = 'blue')
  # geom_area(data = dat3, fill="red", alpha=0.4)

  # geom_line(data = dat3_1, color = 'green') 
  # geom_line(data = dat4_1, color = 'blue') + 
  # geom_line(data = dat5_1, color = 'blue') + 
  # geom_line(data = dat6_1, color = 'blue')
