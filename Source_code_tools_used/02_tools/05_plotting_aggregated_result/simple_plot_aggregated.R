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
set.seed(1)


##############
# configurations
##############
param.rand = TRUE
param.randmode = 'meanpoint' #eachpoint
param.plotmode = FALSE
param.mode.savePlot = TRUE

DEBUG = FALSE

rand_seed_start = 1
rand_seed_end = 9
start_tick = 1
end_tick = 9999

# mode = "max"
# target_drone = "swarm"
param.threshold = -999

param.mode_set = c("min", "max", "avg")
param.target_drone_set = c("swarm") # "d1", "d2", "d3")#, 


for (mode in param.mode_set){
  for (target_drone in param.target_drone_set){
    
    
    
    # base directory
    base_directory = paste("/mnt/storage1/Github/11_SwarmSlicing/Project_data_analysis/",sep="" )
    # directory for input 
    input_directory = paste(base_directory,"input/test/",sep="" )
    # input_directory_randomized_data = paste(input_directory,"data_1_62/",sep="" )
    # directory for output
    # output_directory_randomized_data = paste(base_directory,"output/randomized/",sep="" )
    # output_directory_aligned_data = paste(base_directory,"output/aligned/",sep="" )
    
    # directory for plot-output
    output_directory_plot = paste(base_directory,"output/plot/",sep="" )
    
    
    ##############
    # Reading data
    # refer: https://lightblog.tistory.com/13
    # factor, refer: https://rfriend.tistory.com/32￣
    # which, refer: http://egloos.zum.com/entireboy/v/4837061
    ##############￣
    datalist <- list.files(path = input_directory)
    datalist
    i = 2
    # data_1_50_modified_param
    # east.csv
    
    filename <- datalist[i]
    cat(filename)
    csvname = paste(input_directory,filename,sep="" )
    data_raw <- read.csv(file = csvname, header = TRUE, fileEncoding="UTF-8-BOM", as.is = 1, sep = ",")
    
    # Test, show the first row of data_raw
    data_raw[1,]
    
    
    
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
    
    # cols <- hcl.colors(8, palette = "viridis", alpha = 0.8)
    cols <- c("#004586", "#ff420e", "#ffd320", "#579d1c", "#7e0021", "#83caff", "#314004", "#aecf00")
    output_filename = paste(target_drone, "_", rand_seed_start, "_", rand_seed_end, "_", mode, sep = "")
    
    plotfilename = paste(output_directory_plot, output_filename,".png",sep="" )
    
    if(param.mode.savePlot == TRUE) {png(plotfilename, width = 1500, height = 500, units = "px")}
    
    areaplot(x, y, prop = TRUE, col = cols,
             border = FALSE,
             xlab = "Tick",
             ylab = "Dcc",
             lwd = 1,
             lty = 1)
             # legend = TRUE,
             # args.legend = list(x = "bottomright", cex = 0.65,
             #                    bg = "white", bty = "o"))
    
    if(param.mode.savePlot == TRUE){dev.off()}
    
    
    aggregated_datafilename = paste(output_directory_plot, output_filename,".csv",sep="" )
    write.csv(dcc_adj_data_1[,c(3,4:11)], aggregated_datafilename)



  }
}

# for loop
# endLoop <- length(datalist)
