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

param.mode_set = c("max")#, "min", "avg")
param.target_drone_set = c("d1")#, "d2", "d3", "swarm")

rand_seed_from = 1 # <====== Usually manupulate this
rand_seed_to = 1 # <====== Usually manupulate this
rand_seed_start_set = c(rand_seed_from:rand_seed_to)


for (i in c(5:5)){

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
        
        # Test, show the first row of data_raw
        # data_raw[1,]
        
        #'*CORE HANDLING FUNCTION*#
        dcc_adj_data_1 <- func.handle_dcc(rand_seed_start, rand_seed_end, mode, target_drone)
        
        
        
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Output field
        #
        # Percentage stacked area chart
        # refer: https://r-charts.com/evolution/percentage-stacked-area/
        # palette = "ag_Sunset", "PinkYl", "Mint"
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        x <- dcc_adj_data_1$t
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
        
        # For vertical line indicating slices
        test = c(82)
        abline(v = test, col="black", lwd=3, lty=1)
        
        # appearance / disappearance
        mark_1 = c(344, 354)
        abline(v = mark_1, col="blue", lwd=1, lty=1)
        text(x = mark_1, y = 0.2, label = "R01", srt = 90)
        
        mark_2 = c(312, 336, 363)
        abline(v = mark_2, col="red", lwd=1, lty=1)
        text(x = mark_2, y = 0.3, label = "R02", srt = 90)
        
        # appearance / disappearance with prunned data
        # mark_1_1 = c(82,120,147,181,201,264,273,288,294,308,350,357,3,16,49,54,159,172,200,349,363,12,137,150,151,183,195,201,294,313,321,361)
        mark_1_1 = c(11,149,153,193,201,290,334,340,346)
        abline(v = mark_1_1, col="blue", lwd=1, lty=2)
        text(x = mark_1_1, y = 0.4, label = "R01_1", srt = 90)
        # capture
        
        mark_2_1 = c(34,36,146,190,201,204,215,290,327,334,340,363)
        abline(v = mark_2_1, col="red", lwd=1, lty=2)
        text(x = mark_2_1, y = 0.5, label = "R02_1", srt = 90)
        # capture
        
        
        # not abs taken
        # mark_2 = c(15,32,37,38,156,157,158,202,203,204,205,297,310,313,316,319,325,329,336,338,339,343,350,352,354,357,359,362)
        # abline(v = mark_2, col="white", lwd=3, lty=2)
        
        # abs taken: rapid increase/decrease
        # rapid increase/decrease for all factors: Dcc stream
        # mark_3 = c(10,11,12,15,32,37,38,149,150,151,152,156,157,158,202,203,204,205,290,291,293,296,297,298,301,310,313,316,319,325,326,328,329,330,331,333,336,338,339,343,345,346,348,349,350,351,352,354,357,358,359,360,361,362,363)
        mark_3 = c(5,6,7,8,10,11,12,13,14,150,151,152,153,203,205,291,292,293,294)
        abline(v = mark_3, col="black", lwd=1, lty=1)
        text(x = mark_3, y = 0.6, label = "R03", srt = 90)
        # capture
        
        # merged
        # mark_4 = c(5,6,7,8,9,10,11,12,15,20,32,37,38,149,150,151,152,155,156,157,158,202,203,204,205,290,291,293,294,295,296,297,298,299,300,301,305,306,308,309,310,311,313,314,315,316,319,320,325,326,328,329,330,331,333,336,338,339,343,345,346,348,349,350,351,352,354,357,358,359,360,361,362,363,364)
        mark_4 = c(14, 36, 146, 215, 290)
        abline(v = mark_4, col="white", lwd=3, lty=2)
        text(x = mark_4, y = 0.7, label = "Slice", srt = 90, col = "white")
        
        if(param.mode.savePlot == TRUE){dev.off()}
        
    
    
    
      } # target_drone loop
    } # mode loop
  } # rand_seed_start loop
} # i loop  
