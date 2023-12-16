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
# library(factoextra)

machine <- "Linux"

if(machine == "Mac"){
  base_directory = paste("/Users/XX/Research/Project_data_analysis/",sep="" )
}else if(machine == "Linux"){
  base_directory = paste("/mnt/storage1/Github/11_SwarmSlicing/Project_data_analysis/",sep="" )
}else{
  base_directory = paste("../",sep="" )
}

source(paste(base_directory, "lib/lib_func.R", sep = ""))
source(paste(base_directory, "lib/lib_handling_dcc.R", sep = ""))
source(paste(base_directory, "lib/lib_rules.R", sep = ""))
source(paste(base_directory, "lib/lib_piecewise.R", sep = ""))

# directory for input 
input_directory = paste(base_directory,"QE/M1/input/",sep="" )
datalist <- list.files(path = input_directory)
datalist
i <- 4
filename <- datalist[i]
filename <- "M1_swarm_1_9_avg.csv"
# directory for plot-output
output_directory_plot = paste(base_directory,"QE/M1/output/plot/",filename,"/",sep="" )
dir.create(output_directory_plot)

##############
# Configurations for general setting
##############
set.seed(1)
param.is_prunning <- TRUE
param.unit_test_time <- 50
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
        

        
#' #'*CORE HANDLING FUNCTION*#
#' dcc_adj_data_1 <- func.handle_dcc(rand_seed_start, rand_seed_end, mode, target_drone)
        
        
        
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# Output field
#
# Percentage stacked area chart
# refer: https://r-charts.com/evolution/percentage-stacked-area/
# palette = "ag_Sunset", "PinkYl", "Mint"
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x <- data_raw$t
y <- data_raw[,c(4:11)]

cols <- c("#004586", "#ff420e", "#ffd320", "#579d1c", "#7e0021", "#83caff", "#314004", "#aecf00")

# output_filename = paste(filename,"_", rule, sep = "")
# plotfilename = paste(output_directory_plot, output_filename,".png",sep="" )
# if(param.mode.savePlot == TRUE) {png(plotfilename, width = 4000, height = 500, units = "px")}

areaplot(x, y, prop = TRUE, col = cols,
         border = FALSE,
         xlab = "Tick",
         ylab = "Dcc",
         lwd = 1,
         lty = 1
         )
        
        # legend = TRUE,
        # args.legend = list(x = "bottomright", cex = 0.65,
        #                     bg = "white", bty = "o"))

# 
# line graph graphical options
# https://www.statmethods.net/advgraphs/parameters.html
# 
#'*DATA PROCESSING*
# 
source(paste(base_directory, "lib/lib_rules.R", sep = ""))
rule_mode <- "rule01"
slice_after_rule_1 <- func.apply_rules(data_raw, rule_mode, 5, FALSE)#param.is_prunning)
abline(v = slice_after_rule_1, col="blue", lwd=2, lty=1)


rule_mode <- "rule02"
slice_after_rule_2 <- func.apply_rules(data_raw, rule_mode, 5, FALSE)#param.is_prunning)
abline(v = slice_after_rule_2, col="red", lwd=2, lty=1)
slice_after_rule_2

# Unused code below
if(FALSE){
rule_mode <- "rule03"
slice_after_rule_3 <- func.apply_rules(data_raw, rule_mode, 5, param.is_prunning)
abline(v = slice_after_rule_3, col="black", lwd=1, lty=1)

slice_after_rule_sum <- c(slice_after_rule_1, slice_after_rule_2, slice_after_rule_3)

slice_after_rule <- unique(sort(slice_after_rule_sum))
} # end of comment

# write.csv(slice_after_rule, paste(input_directory, "temp_output.csv"))


#'*Piecewise Linear Function* 
#'Before merge

# little pre-processing
# This is because data_raw is literraly raw data so not fitted to the accumulated data
# This process is transforming raw data (delta) to processed data (dcc).
# 
# data_raw2 is accumulated function
# 

data_raw2 <- data_raw

for (row_idx in c(1:nrow(data_raw))){
  for (col_idx in c(4:11)){
    data_raw2[row_idx,col_idx] <- data_raw[row_idx,col_idx] / sum(data_raw[row_idx,4:11])
    if(col_idx >= 5){
      data_raw2[row_idx,col_idx] <- data_raw2[row_idx, (col_idx - 1)] + data_raw2[row_idx, col_idx]
    }
  }
}


#
#'*CORE FUNCTION: func.piecewise()*
# Also 8 is the num of breakpoint
# 
slice_after_rule_3 <- NULL

# 
# This part is for 'des' which is data_raw2[,4]
# which means (1/8).
# 

dat1 <- data.frame(x = data_raw2$t, y = data_raw2[,4])
o1 <- func.piecewise(dat1, 10)
dat1_1 <- data.frame(x = dat1$x, y = broken.line(o1)$fit)

lines(dat1_1, col = "white", lwd = 3, lty = 1)
abline(v = o1$psi[,2], col = "white", lwd = 1, lty = 2)

# 
# This part is for 'leader' which is data_raw2[,5]
# which means (2/8).
# 

dat2 <- data.frame(x = data_raw2$t, y = data_raw2[,5])
o2 <- func.piecewise(dat2, 9)
dat2_1 <- data.frame(x = dat2$x, y = broken.line(o2)$fit)

lines(dat2_1, col = "white", lwd = 3, lty = 1)
abline(v = o2$psi[,2], col = "white", lwd = 1, lty = 2)

# 
# This part is for 'f1' which is data_raw2[,6]
# which means (3/8).
# 

dat3 <- data.frame(x = data_raw2$t, y = data_raw2[,6])
o3 <- func.piecewise(dat3, 8)
dat3_1 <- data.frame(x = dat3$x, y = broken.line(o3)$fit)

lines(dat3_1, col = "white", lwd = 3, lty = 1)
abline(v = o3$psi[,2], col = "white", lwd = 1, lty = 2)

# 
# This part is for 'des' which is data_raw2[,7]
# which means (4/8).
# 

dat4 <- data.frame(x = data_raw2$t, y = data_raw2[,7])
o4 <- func.piecewise(dat4, 10) # when we set 8, error occurs, so I set this as 4
dat4_1 <- data.frame(x = dat4$x, y = broken.line(o4)$fit)

lines(dat4_1, col = "white", lwd = 3, lty = 1)
abline(v = o4$psi[,2], col = "white", lwd = 1, lty = 2)

# 
# This part is for 'des' which is data_raw2[,8]
# which means (5/8).
# 
# #####################################################
# BELOW is NOT NEEDED ACTUALLY!
# #####################################################
if(FALSE){
dat5 <- data.frame(x = data_raw2$t, y = data_raw2[,8])
o5 <- func.piecewise(dat5, 8)
dat5_1 <- data.frame(x = dat5$x, y = broken.line(o5)$fit)

lines(dat5_1, col = "white", lwd = 3, lty = 1)
abline(v = o5$psi[,2], col = "red", lwd = 3, lty = 2)

# 
# This part is for 'des' which is data_raw2[,9]
# which means (6/8).
# 

dat6 <- data.frame(x = data_raw2$t, y = data_raw2[,9])
o6 <- func.piecewise(dat6, 8)
dat6_1 <- data.frame(x = dat6$x, y = broken.line(o6)$fit)

lines(dat6_1, col = "white", lwd = 3, lty = 1)
abline(v = o6$psi[,2], col = "red", lwd = 3, lty = 2)

# 
# This part is for 'des' which is data_raw2[,10]
# which means (7/8).
# 

dat7 <- data.frame(x = data_raw2$t, y = data_raw2[,10])
o7 <- func.piecewise(dat7, 8)
dat7_1 <- data.frame(x = dat7$x, y = broken.line(o7)$fit)

lines(dat7_1, col = "white", lwd = 3, lty = 1)
abline(v = o7$psi[,2], col = "red", lwd = 3, lty = 2)

# 
# This part is for 'des' which is data_raw2[,11]
# which means (8/8).
# 

dat8 <- data.frame(x = data_raw2$t, y = data_raw2[,11])
o8 <- func.piecewise(dat8, 8)
dat8_1 <- data.frame(x = dat8$x, y = broken.line(o8)$fit)

lines(dat8_1, col = "white", lwd = 3, lty = 1)
abline(v = o8$psi[,2], col = "red", lwd = 3, lty = 2)



slice_after_rule_3 <- c(o1$psi[,2], o2$psi[,2], o3$psi[,2], o4$psi[,2],
                        o5$psi[,2], o6$psi[,2], o7$psi[,2], o8$psi[,2])

} # end of comment




slice_after_rule_3 <- c(o1$psi[,2], o2$psi[,2], o3$psi[,2], o4$psi[,2])

slice_final <- c(slice_after_rule_1, slice_after_rule_2, slice_after_rule_3)

slice_final <- unique(sort(slice_final))
# Merge

representitive_boundary <- func.repres_boundary( slice_final, param.unit_test_time )

abline(v = representitive_boundary, col="black", lwd=4, lty=1)


if(param.mode.savePlot == TRUE){dev.off()}
        
        
        
