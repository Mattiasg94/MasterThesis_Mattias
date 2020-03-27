



clc

w=[4.5503e+000     8.0010e+000    11.2507e+000    13.7687e+000    15.8560e+000    19.1200e+000];
IR_loss=[1.4829e+000     1.9323e+000     2.3864e+000     2.8574e+000     3.3385e+000     3.7922e+000];

Fit = polyfit(w,IR_loss,1); % x = x data, y = y data, 1 = order of the polynomial i.e a straight line 
IR_loss_est=polyval(Fit,w)