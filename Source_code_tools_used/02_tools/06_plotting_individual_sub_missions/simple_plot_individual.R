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

source("/mnt/storage1/Github/11_SwarmSlicing/Project_data_analysis/lib/lib_func.R")
source("/mnt/storage1/Github/11_SwarmSlicing/Project_data_analysis/lib/lib_handling_dcc.R")


##############
# Configurations for general setting
##############
set.seed(1)
param.mode.savePlot = TRUE # Usually TRUE as this program is for getting plots.
param.isCrash = FALSE # If there is crash case in random seed so you need to avoid it, flip this to TRUE.

DEBUG = FALSE

##############
# Parameters for getting data
##############
start_tick = 1
end_tick = 9999

param.threshold = -999

param.mode_set = c("max", "min", "avg")
param.target_drone_set = c("d1", "d2", "d3", "swarm")

rand_seed_from = 1 # <====== Usually manupulate this
rand_seed_to = 10 # <====== Usually manupulate this
rand_seed_start_set = c(rand_seed_from:rand_seed_to)


for (i in c(1:1)){

  for (rand_seed_start in rand_seed_start_set){
    rand_seed_end = rand_seed_start#79
  
    for (mode in param.mode_set){
      for (target_drone in param.target_drone_set){
        
        
        
        ##############
        # Reading data
        # refer: https://lightblog.tistory.com/13
        # factor, refer: https://rfriend.tistory.com/32￣
        # which, refer: http://egloos.zum.com/entireboy/v/4837061
        ##############￣
        
        # base directory
        base_directory = paste("/mnt/storage1/Github/11_SwarmSlicing/Project_data_analysis/",sep="" )
        # directory for input 
        input_directory = paste(base_directory,"input/test/",sep="" )
        
        datalist <- list.files(path = input_directory)
        filename <- datalist[i]
        
        # directory for plot-output
        output_directory_plot = paste(base_directory,"output/plot/",filename,"/",sep="" )
        dir.create(output_directory_plot)
        
        cat("Current file is [", filename, "].\n")
        csvname = paste(input_directory,filename,sep="" )
        data_raw <- read.csv(file = csvname, header = TRUE, fileEncoding="UTF-8-BOM", as.is = 1, sep = ",")
        

        #'*CORE HANDLING FUNCTION*#
        dcc_adj_data_1 <- func.handle_dcc(rand_seed_start, rand_seed_end, mode, target_drone)
        
        
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Output field
        #
        # Percentage stacked area chart
        # refer: https://r-charts.com/evolution/percentage-stacked-area/
        # palette = "ag_Sunset", "PinkYl", "Mint"
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        # x <- dcc_adj_data_1$t
        x <- c(1:length(dcc_adj_data_1$t))
        y <- dcc_adj_data_1[,c(4:11)]
        
        cols <- c("skyblue", "red", "blue", "green", "brown", "black", "orange", "yellow")
        output_filename = paste(target_drone, "_", rand_seed_start, "_", rand_seed_end, "_", mode, sep = "")
        
        plotfilename = paste(output_directory_plot, output_filename,".png",sep="" )
        
        if(param.mode.savePlot == TRUE) {png(plotfilename, width = 4000, height = 500, units = "px")}
        
        areaplot(x, y, prop = TRUE, col = cols,
                 border = "black",
                 xlab = "Tick",
                 ylab = "Dcc",
                 lwd = 1,
                 lty = 1,
                 legend = TRUE,
                 args.legend = list(x = "bottomright", cex = 0.65,
                                    bg = "white", bty = "o"))
        # 
        # line graph graphical options
        # https://www.statmethods.net/advgraphs/parameters.html
        #
        
        
        
        if(param.mode.savePlot == TRUE){dev.off()}
        
    
    
    
      } # target_drone loop
    } # mode loop
  } # rand_seed_start loop
} # i loop  
