param.unit_test_time <- 50


# [1]   0.00000   5.00000  21.00000  34.00000  36.00000  42.00000  43.09783  56.91316  59.00000  66.57902
# ...
slice_final

# [1]   5.00000  21.00000  34.00000  36.00000  42.00000  43.09783  56.91316  59.00000  66.57902  75.77841
# ...
one_less_slided <- slice_final[-1]
one_less_slided <- c(one_less_slided, 0)

# [1]   0.00000   0.00000   5.00000  21.00000  34.00000  36.00000  42.00000  43.09783  56.91316  59.00000
# ...
one_more_slided <- c(0, slice_final)
one_more_slided <- one_more_slided[-length(one_more_slided)]

gap_previous <- abs(slice_final - one_more_slided)
gap_next <- abs(slice_final - one_less_slided)
  

gap_previous
gap_next
  
mydata <- data.frame(slice_final, gap_previous, gap_next)  

det_con_1 <- which(gap_previous >= param.unit_test_time )
det_con_2 <- which(gap_next >= param.unit_test_time)

det_slice_boundary <- c (det_con_1, det_con_2)
det_slice_boundary

mydata$slice_final[ det_slice_boundary ]

abline(v = mydata$slice_final[ det_slice_boundary ], col="blue", lwd=1, lty=1)

