func.apply_rules <- function(table, rule_mode, offset1, isPrunning){
  #'*Arguments definition*
  # INPUT: [matrix/table]
  # INPUT: mode = {"rule01", "rule02", "rule03", "rule04",...}
  # OUTPUT: [array]: the place of the cut
  # 
  
  # TODO: should be taken from outside
  params.thres.prunning <- 0.05
  params.thres.gradient <- 0.01
  
  
  if (isPrunning == TRUE){
    
    for (row_idx in c(1:nrow(table))){
      table[row_idx, which(table[row_idx,] <= params.thres.prunning)] <- 0
    }
  }
  
  if (rule_mode == "rule03"){
    table_gradient <- table
    
    # For take gradient of table given
    for (row_idx2 in c(2:nrow(table))){
      table_gradient[row_idx2,] <- table[row_idx2,] - table[(row_idx2 - 1),]
    }
    table_gradient[1,] <- 0
    
  }
  
  
  thres_list <- 0
  
  # TODO offset also should be given by outside of this function
  offset <- offset1 #for small window
  offset_wide_win <- 5 #2 # for wide window
  
  
  # MAIN LOOP
  # for loop for row
  #for (row_idx in c(start_no: nrow(table))){
  for (row_idx in c(1: nrow(table))){
    
    
    isThres <- FALSE
    det <- FALSE
    isRowBoundary <- FALSE
    
    # for loop for col
    for (col_idx in c(4:11)){
      
      # CORE LOOP
      # "== 5"  means all cells before the targeting row are all 0.
      
      
      # Rule for appreance
      if(rule_mode == "rule01"){
        
        # pass condition which differs from mode
        if(row_idx <= offset){
          next
        }
        
        # targetting row should be empty (== less than the threshold)
        # and also 5 elements before the targetting row should be empty as well.

        det <- sum(table[c( (row_idx-offset) : (row_idx-1)), col_idx] == 0) == 5 && (table[row_idx, col_idx] != 0)
        
      # Rule for disappreance  
      }else if(rule_mode == "rule02"){
        
        # pass condition which differs from mode
        # To mark the end of sim
        if (row_idx == nrow(table)) {
          cat(row_idx, "] det <- TRUE \n")
          det <- TRUE
        }else if(row_idx >= nrow(table) - offset){
          next
        }
        
        if (row_idx != nrow(table)) {
          det <- sum(table[c( (row_idx+1) : (row_idx+offset)), col_idx] == 0) == 5 && (table[row_idx, col_idx] != 0)  
        }
        
        
        
      # Rule for rapid increase/decrease  
      }else if(rule_mode == "rule03"){
        
        # pass condition which differs from mode
        if(row_idx <= offset){
          next
        }
        
        
        
        # 2nd trial
        # avg_gradient <- 0.25 * (table_gradient[(row_idx - offset), col_idx] - table_gradient[(row_idx - 1), col_idx])
        

        
        # 3rd trial 
        # Wide window
        front_value_idx_2 <- func.prev((row_idx - offset_wide_win), nrow(table))
        rear_value_idx_2 <- func.prev((row_idx + offset_wide_win), nrow(table))

        avg_gradient <- ( 1 /  ( rear_value_idx_2 - front_value_idx_2 )) * ( table[front_value_idx_2 , col_idx] - table[rear_value_idx_2 , col_idx] )


        det <- abs(avg_gradient) >= params.thres.gradient

        if (row_idx == 225){
          cat("[dbg1] ", abs(avg_gradient))
        }
        
        # Then small window (i.e., gradient for each tick)
        if (det) {
          front_value_idx_1 <- func.prev((row_idx - offset), nrow(table))
          rear_value_idx_1 <- func.prev((row_idx + offset), nrow(table))

          avg_gradient <- mean(table_gradient[front_value_idx_1 : rear_value_idx_1, col_idx])
          det <- abs(avg_gradient) >= params.thres.gradient
        }
          

        

        
      }
      
      
      
      if ( det ){
        isRowBoundary <- TRUE
      }  
      
    }
      
    if (isRowBoundary){
      thres_list <- c(thres_list, row_idx)
    }

  }
  
  res <- thres_list
  return(res)
}


func.prev <- function(desired_point, max_tick){
  
  res <- desired_point
  
  if(desired_point < 1){
    res <- 1
  }else if(desired_point >= max_tick){
    res <- max_tick
  }
  
  return(res)
}

func.repres_boundary <- function(boundary_list, parameter_unit_test_cost){
  
  
  # [1]   0.00000   5.00000  21.00000  34.00000  36.00000  42.00000  43.09783  56.91316  59.00000  66.57902
  # ...
  boundary_list
  
  # [1]   5.00000  21.00000  34.00000  36.00000  42.00000  43.09783  56.91316  59.00000  66.57902  75.77841
  # ...
  one_less_slided <- boundary_list[-1]
  one_less_slided <- c(one_less_slided, 0)
  
  # [1]   0.00000   0.00000   5.00000  21.00000  34.00000  36.00000  42.00000  43.09783  56.91316  59.00000
  # ...
  one_more_slided <- c(0, boundary_list)
  one_more_slided <- one_more_slided[-length(one_more_slided)]
  
  gap_previous <- abs(boundary_list - one_more_slided)
  gap_next <- abs(boundary_list - one_less_slided)
  
  
  gap_previous
  gap_next
  
  mydata <- data.frame(boundary_list, gap_previous, gap_next)  
  
  det_con_1 <- which(gap_previous >= parameter_unit_test_cost )
  det_con_2 <- which(gap_next >= parameter_unit_test_cost)
  
  det_slice_boundary <- c (det_con_1, det_con_2)
  det_slice_boundary
  
  res <- mydata$boundary_list[ det_slice_boundary ]
  
  # abline(v = mydata$slice_final[ det_slice_boundary ], col="blue", lwd=1, lty=1)
  
  return(res)
}
