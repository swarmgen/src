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
source("/home/XX/91_data_storage_mt/Github/11_SwarmSlicing/Project_data_analysis/lib/lib_func.R")

set.seed(1)


##############
# configurations
##############
param.rand = TRUE
param.randmode = 'meanpoint' #eachpoint
param.plotmode = FALSE
param.mode.savePlot = TRUE

DEBUG = FALSE

rand_seed_start = 11
rand_seed_end = rand_seed_start#79
start_tick = 1
end_tick = 500

# mode = "max"
# target_drone = "swarm"
param.threshold = -999

param.mode_set = c("max")#, "min")
param.target_drone_set = c("d1", "d2", "d3", "swarm")

rand_seed_start_set = c(1:79)

for (rand_seed_start in rand_seed_start_set){
  rand_seed_end = rand_seed_start#79
  


for (mode in param.mode_set){
  for (target_drone in param.target_drone_set){
    
    
    
    # base directory
    base_directory = paste("/home/XX/91_data_storage_mt/Github/11_SwarmSlicing/Project_data_analysis/",sep="" )
    # directory for input 
    input_directory = paste(base_directory,"input/",sep="" )
    # input_directory_randomized_data = paste(input_directory,"data_1_62/",sep="" )
    # directory for output
    # output_directory_randomized_data = paste(base_directory,"output/randomized/",sep="" )
    # output_directory_aligned_data = paste(base_directory,"output/aligned/",sep="" )
    
    # directory for plot-output
    output_directory_plot = paste(base_directory,"output/plot/",sep="" )
    
    
    ##############
    # Reading data
    # refer: https://lightblog.tistory.com/13
    # factor, refer: https://rfriend.tistory.com/32
    # which, refer: http://egloos.zum.com/entireboy/v/4837061
    ##############
    datalist <- list.files(path = input_directory)
    i = 1 # second file: data_61_70.csv
    filename <- datalist[i]
    filename
    csvname = paste(input_directory,filename,sep="" )
    data_raw <- read.csv(file = csvname, header = TRUE, fileEncoding="UTF-8-BOM", as.is = 1, sep = ",")
    
    # Test, show the first row of data_raw
    data_raw[1,]
    
    
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # temp_data[,2] # Drone index, we have 1,2 and 3 (not have leader yet)
    # temp_data[,3] # Tick
    # temp_data[,4:11] # Delta values
    # temp_data[,12:19] # Dcc values
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for(radn_seed_index in rand_seed_start:rand_seed_end){
      # select rows that have value 1 in randomseed column
      # radn_seed_index <- 69
      temp_data <- data_raw[which(data_raw$randomseed == radn_seed_index),]
      temp_data_1 <- temp_data[which(temp_data$drone_no == 1),]
      temp_data_2 <- temp_data[which(temp_data$drone_no == 2),]
      temp_data_3 <- temp_data[which(temp_data$drone_no == 3),]
      
      # For initial table
      # when only radn_seed_index == first seed num
      if (radn_seed_index == rand_seed_start) {
        if (target_drone == "d1"){
          adj_data_1 <- temp_data_1
          
        }else if (target_drone == "d2"){
          adj_data_1 <- temp_data_2
    
        }else if (target_drone == "d3"){
          adj_data_1 <- temp_data_3
    
        }else if (target_drone == "swarm"){
          
          adj_data_1 <- func.merge(temp_data_1, temp_data_2, start_tick, end_tick, mode, param.threshold)
          adj_data_1 <- func.merge(adj_data_1, temp_data_3, start_tick, end_tick, mode, param.threshold)
          
        }
      }
      
      if (target_drone == "d1"){
        temp_comparison <- temp_data_1
      }else if (target_drone == "d2"){
        temp_comparison <- temp_data_2
      }else if (target_drone == "d3"){
        temp_comparison <- temp_data_3
      }else if (target_drone == "swarm"){
        
        temp_comparison <- func.merge(temp_data_1, temp_data_2, start_tick, end_tick, mode, param.threshold)
        temp_comparison <- func.merge(adj_data_1, temp_data_3, start_tick, end_tick, mode, param.threshold)
        
      }
      
      # Usage: function(table1, table2, start_tick, end_tick, mode, threshold)
      adj_data_1 <- func.merge(adj_data_1, temp_comparison, start_tick, end_tick, mode, param.threshold)
      
      
    }
    
    
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Generating Dcc field
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    dcc_adj_data_1 <- adj_data_1
    
    for(k in 4:11){
      for(j in 1:290){
        
        dcc_adj_data_1[j,k] <- adj_data_1[j,k] / sum(adj_data_1[j,c(4:11)])
        
        if(DEBUG){cat(adj_data_1[j,k], " is adj_data_1[j,k]", temp_data_1[j,k], " is temp_data_1[j,k] \n" )}
        
      }
    }
    
    
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Output field
    #
    # Percentage stacked area chart
    # refer: https://r-charts.com/evolution/percentage-stacked-area/
    # palette = "ag_Sunset", "PinkYl", "Mint"
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    x <- dcc_adj_data_1$t
    y <- dcc_adj_data_1[,c(4:11)]
    
    cols <- hcl.colors(8, palette = "viridis", alpha = 0.8)
    output_filename = paste(target_drone, "_", rand_seed_start, "_", rand_seed_end, "_", mode, sep = "")
    
    plotfilename = paste(output_directory_plot, output_filename,".png",sep="" )
    
    if(param.mode.savePlot == TRUE) {png(plotfilename, width = 1000, height = 500, units = "px")}
    
    areaplot(x, y, prop = TRUE, col = cols,
             border = "black",
             xlab = "Tick",
             ylab = "Dcc",
             lwd = 1,
             lty = 1,
             legend = TRUE,
             args.legend = list(x = "bottomright", cex = 0.65,
                                bg = "white", bty = "o"))
    
    if(param.mode.savePlot == TRUE){dev.off()}



  }
}

}
# for loop
# endLoop <- length(datalist)
