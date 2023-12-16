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
source(paste(base_directory, "lib/lib_rules.R", sep = ""))
source(paste(base_directory, "lib/lib_piecewise.R", sep = ""))

# directory for input 
input_directory = paste(base_directory,"input/applying_rules/",sep="" )
datalist <- list.files(path = input_directory)
datalist
i <- 3
filename <- datalist[i]

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
        
        cols <- c("skyblue", "red", "blue", "green", "brown", "black", "orange", "yellow")
        output_filename = paste(filename,"_", rule, sep = "")
        
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
        #'*DATA PROCESSING*
        # 
        source(paste(base_directory, "lib/lib_rules.R", sep = ""))
        rule_mode <- "rule01"
        slice_after_rule_1 <- func.apply_rules(data_raw, rule_mode, 5, param.is_prunning)
        abline(v = slice_after_rule_1, col="blue", lwd=1, lty=1)
        
        rule_mode <- "rule02"
        slice_after_rule_2 <- func.apply_rules(data_raw, rule_mode, 5, param.is_prunning)
        abline(v = slice_after_rule_2, col="red", lwd=1, lty=1)
        
        rule_mode <- "rule03"
        slice_after_rule_3 <- func.apply_rules(data_raw, rule_mode, 1, param.is_prunning)
        abline(v = slice_after_rule_3, col="black", lwd=1, lty=1)
        
        slice_after_rule_sum <- c(slice_after_rule_1, slice_after_rule_2, slice_after_rule_3)
        
        slice_after_rule <- unique(sort(slice_after_rule_sum))
        
        # write.csv(slice_after_rule, paste(input_directory, "temp_output.csv"))
        
        
        #'*Piecewise Linear Function* 
        #'Before merge
        
        # little pre-processing
        # This is because data_raw is literraly raw data so not fitted to the accumulated data
        # This process is transforming raw data (delta) to processed data (dcc).
        
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
        # This part is for 'des' which is data_raw2[,4]
        # which means (1/8).
        # 
        
        dat1 <- data.frame(x = data_raw2$t, y = data_raw2[,4])
        o1 <- func.piecewise(dat1, 8)
        dat1_1 <- data.frame(x = dat1$x, y = broken.line(o1)$fit)
        
        lines(dat1_1, col = "white", lwd = 3, lty = 1)
        abline(v = o1$psi[,2], col = "red", lwd = 3, lty = 2)
        
        # 
        # This part is for 'leader' which is data_raw2[,5]
        # which means (2/8).
        # 
        
        dat2 <- data.frame(x = data_raw2$t, y = data_raw2[,5])
        o2 <- func.piecewise(dat2, 8)
        dat2_1 <- data.frame(x = dat2$x, y = broken.line(o2)$fit)
        
        lines(dat2_1, col = "white", lwd = 3, lty = 1)
        abline(v = o2$psi[,2], col = "red", lwd = 3, lty = 2)
        
        # 
        # This part is for 'f1' which is data_raw2[,6]
        # which means (3/8).
        # 
        
        dat3 <- data.frame(x = data_raw2$t, y = data_raw2[,6])
        o3 <- func.piecewise(dat3, 8)
        dat3_1 <- data.frame(x = dat3$x, y = broken.line(o3)$fit)
        
        lines(dat3_1, col = "white", lwd = 3, lty = 1)
        abline(v = o3$psi[,2], col = "red", lwd = 3, lty = 2)
        
        # 
        # This part is for 'des' which is data_raw2[,7]
        # which means (4/8).
        # 
        
        dat4 <- data.frame(x = data_raw2$t, y = data_raw2[,7])
        o4 <- func.piecewise(dat4, 4) # when we set 8, error occurs, so I set this as 4
        dat4_1 <- data.frame(x = dat4$x, y = broken.line(o4)$fit)
        
        lines(dat4_1, col = "black", lwd = 3, lty = 1)
        abline(v = o4$psi[,2], col = "red", lwd = 3, lty = 2)
        
        # 
        # This part is for 'des' which is data_raw2[,8]
        # which means (5/8).
        # 
        
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
        
        
        # Merge
        
        
        # slice_after_rule_final <- c(38, 146, 223, 286, 295, 315)
        slice_after_rule_final <- c(0,38,146,223,286,412,458,486,510,547,572,597,705,747,803,906,929,979,1104,1130,1198,1271,1310,1378,1492,1572,1608,1712,1796,1835,1983,2011,2033,2138,2219,2245,2323,2453,2557,2593,2630,2671,2764,2863,2898,3012,3079,3132,3230,3320,3474,3499,3591,3696,3798,3884,3906,4071,4170,4221,4272,4406,4538)
        abline(v = slice_after_rule_final, col="white", lwd=3, lty=2)
        text(x = slice_after_rule, y = 0.2, label = "R01", srt = 90)
        
        
        #' #'*TEST*
        #' # For vertical line indicating slices
        #' test = c(82)
        #' abline(v = test, col="black", lwd=3, lty=1)

        
        #' 
        #' #'*Rule 01* Appearance / Disappearance of the value
        #' # appearance / disappearance
        #' mark_1 = c(344, 354)
        #' abline(v = mark_1, col="blue", lwd=1, lty=1)
        #' text(x = mark_1, y = 0.2, label = "R01", srt = 90)
        #' 
        #' #'*Rule 02* Desppearance of the value
        #' mark_2 = c(312, 336, 363)
        #' abline(v = mark_2, col="red", lwd=1, lty=1)
        #' text(x = mark_2, y = 0.3, label = "R02", srt = 90)
        #' 
        #' #'*Rule 04* Prunning values below the threshold 
        #' # appearance / disappearance with prunned data
        #' # mark_1_1 = c(82,120,147,181,201,264,273,288,294,308,350,357,3,16,49,54,159,172,200,349,363,12,137,150,151,183,195,201,294,313,321,361)
        #' mark_1_1 = c(11,149,153,193,201,290,334,340,346)
        #' abline(v = mark_1_1, col="blue", lwd=1, lty=2)
        #' text(x = mark_1_1, y = 0.4, label = "R01_1", srt = 90)
        #' # capture
        #' 
        #' mark_2_1 = c(34,36,146,190,201,204,215,290,327,334,340,363)
        #' abline(v = mark_2_1, col="red", lwd=1, lty=2)
        #' text(x = mark_2_1, y = 0.5, label = "R02_1", srt = 90)
        #' # capture
        #' 
        #' 
        #' # not abs taken
        #' # mark_2 = c(15,32,37,38,156,157,158,202,203,204,205,297,310,313,316,319,325,329,336,338,339,343,350,352,354,357,359,362)
        #' # abline(v = mark_2, col="white", lwd=3, lty=2)
        #' 
        #' # abs taken: rapid increase/decrease
        #' # rapid increase/decrease for all factors: Dcc stream
        #' # mark_3 = c(10,11,12,15,32,37,38,149,150,151,152,156,157,158,202,203,204,205,290,291,293,296,297,298,301,310,313,316,319,325,326,328,329,330,331,333,336,338,339,343,345,346,348,349,350,351,352,354,357,358,359,360,361,362,363)
        #' 
        #' #'*Rule 3* Rapid increase/decrease of Dcc values
        #' mark_3 = c(5,6,7,8,10,11,12,13,14,150,151,152,153,203,205,291,292,293,294)
        #' abline(v = mark_3, col="black", lwd=1, lty=1)
        #' text(x = mark_3, y = 0.6, label = "R03", srt = 90)
        #' # capture
        #' 
        #' 
        #' 
        #' # merged
        #' # mark_4 = c(5,6,7,8,9,10,11,12,15,20,32,37,38,149,150,151,152,155,156,157,158,202,203,204,205,290,291,293,294,295,296,297,298,299,300,301,305,306,308,309,310,311,313,314,315,316,319,320,325,326,328,329,330,331,333,336,338,339,343,345,346,348,349,350,351,352,354,357,358,359,360,361,362,363,364)
        #' #'*Rule 5* Select representative value from the clusters of cuts
        #' mark_4 = c(14, 36, 146, 215, 290)
        #' abline(v = mark_4, col="white", lwd=3, lty=2)
        #' text(x = mark_4, y = 0.7, label = "Slice", srt = 90, col = "white")
        
        if(param.mode.savePlot == TRUE){dev.off()}
        
        
        
