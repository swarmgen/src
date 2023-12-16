#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# threshold is in R
# when you want disable threshold, set it as 999
# mode is in {min, max}
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

func.proc <- function(value1, value2, mode, threshold){
  temp_value1 <- -999
  temp_value2 <- -999
  res <- -999
  
  temp_value1 <- func.thres(value = value1, threshold = threshold)
  temp_value2 <- func.thres(value = value2, threshold = threshold)
  
  
  
  if(mode == "max"){
    res <- max(temp_value1, temp_value2, na.rm = TRUE)  
  }else if(mode == "min"){
    res <- min(temp_value1, temp_value2, na.rm = TRUE)  
  }
  
  return(res)
}


#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# When input comes in, 
# 1. makes it zero if it is less than the threshold.
# 2. ...
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

func.thres <- function(value, threshold){
  
  adjusted_value <- value
  if(is.na(value)){
   return(adjusted_value)
  }
  if(value <= threshold){
    adjusted_value <- 0
  }
  return(adjusted_value)
}


#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# Merge function
# when two tables come in, return processed table
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

func.merge <- function(table1, table2, start_tick, end_tick, mode, threshold){
  
  res_table <- table1
  
  for(k in 4:11){
    
    # latter number means the maximum tick of simulated exp. need to adjust
    for(j in start_tick:end_tick){ 
      
      # Usage: function(value1, value2, mode, threshold)
      res_table[j,k] <- func.proc(table1[j,k], table2[j,k], mode, threshold)
    }
  }  
  
  return(res_table)
}

