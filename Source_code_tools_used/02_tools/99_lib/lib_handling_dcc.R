#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# threshold is in R
# when you want disable threshold, set it as 999
# mode is in {min, max}
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

func.handle_dcc <- function(rand_seed_start, rand_seed_end, mode, target_drone){
  
  
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  # temp_data[,2] # Drone index, we have 1,2 and 3 (not have leader yet)
  # temp_data[,3] # Tick
  # temp_data[,4:11] # Delta values
  # temp_data[,12:19] # Dcc values
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  for(radn_seed_index in rand_seed_start:rand_seed_end){
    cat("[log] radn_seed_index: ", radn_seed_index, "\n")
    # select rows that have value 1 in randomseed column
    # radn_seed_index <- 69
    
    #'*this some colorful comment*
    # TODO extract below block outside of this function
    #   
    # When some case is crash case.
    #if (param.isCrash && radn_seed_index == 7){
    #  next
    #}
    
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
        
        if (mode == "avg"){
          
          adj_data_1 <- matrix(0, end_tick, ncol(temp_data_1))
          
        }else{
          
          adj_data_1 <- func.merge(temp_data_1, temp_data_2, start_tick, end_tick, mode, param.threshold)
          adj_data_1 <- func.merge(adj_data_1, temp_data_3, start_tick, end_tick, mode, param.threshold)  
          
        }
        
        
      }
    }
    
    if (target_drone == "d1"){
      temp_comparison <- temp_data_1
    }else if (target_drone == "d2"){
      temp_comparison <- temp_data_2
    }else if (target_drone == "d3"){
      temp_comparison <- temp_data_3
    }else if (target_drone == "swarm"){
      
      if (mode == "avg"){
        
        temp_comparison <- (temp_data_1[start_tick:end_tick,] + temp_data_2[start_tick:end_tick,] + temp_data_3[start_tick:end_tick,]) / 3 # take the average
        
      }else{
        
        temp_comparison <- func.merge(temp_data_1, temp_data_2, start_tick, end_tick, mode, param.threshold)
        temp_comparison <- func.merge(adj_data_1, temp_data_3, start_tick, end_tick, mode, param.threshold)  
        
      }
      
    }
    
    cat("[log] radn_seed_index2: ", radn_seed_index, "\n")
    
    # Usage: function(table1, table2, start_tick, end_tick, mode, threshold)
    if (mode == "avg"){
      
      adj_data_1 <- adj_data_1[start_tick:end_tick,] + temp_comparison[start_tick:end_tick,] # accumulate for the numerator
      
    }else{
      adj_data_1 <- func.merge(adj_data_1, temp_comparison, start_tick, end_tick, mode, param.threshold)  
    }
    
  }
  
  cat("[log] check point 02 \n")
  
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  # Generating Dcc field
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if (mode == "avg"){
    temp_denominator <- rand_seed_end - rand_seed_start + 1
    dcc_adj_data_1 <- adj_data_1 /  temp_denominator # divide by the denominator
    
  }else{
    
    dcc_adj_data_1 <- adj_data_1  
    
  }
  
  for(k in 4:11){
    for(j in 1:nrow(adj_data_1)){
      
      dcc_adj_data_1[j,k] <- adj_data_1[j,k] / sum(adj_data_1[j,c(4:11)])
      
      if(DEBUG){cat(adj_data_1[j,k], " is adj_data_1[j,k]", temp_data_1[j,k], " is temp_data_1[j,k] \n" )}
      
    }
  }
  
  
  
  return(dcc_adj_data_1)
}