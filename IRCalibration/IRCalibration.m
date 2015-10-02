function result = IRCalibration()

    ohms =  [1150 2870 4750 10000 20000];
    difference = [290 740 870 786 684];

    hold on

    plot(ohms, difference, 'linewidth', 2)

%     p = polyfit(ohms, difference, 3);
% 
%     y1 = polyval(p, ohms);
% 
%     plot(ohms, y1, 'r', 'linewidth', 2)
    
%     disp(p);
%     
%     fh = @(x) p(1) * x^2 + p(2) * x + p(3);
%     ezplot(fh, ohms);
    xlabel('R\_Sense Resistance Value')
    ylabel('Difference Between White and Black IR Readings')
%     legend('Our Measurements', 'Line of Best Fit')
    title('IR Sensor Resistor Calibration')

end