#install.packages("segmented")
library(segmented)
library(ggplot2)
#install.packages("ggplot2")


func.piecewise <- function(data_frame, num_breakpoint){
  



dati <- data_frame
out.lm <- lm(y ~ x, data = dati)
o <- segmented(out.lm, segZ = ~x, npsi = num_breakpoint, control = seg.control(display = FALSE))
# dat2 <- data.frame(x = dati$x, y = broken.line(o)$fit)

  
 return(o)
}