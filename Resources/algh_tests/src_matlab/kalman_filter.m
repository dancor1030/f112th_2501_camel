clc
clear
close all
%%

data = readtable("DEFAULT0.csv")

%%
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

y = filter(b,a,data.TimeBase_200ms);

plot((20:length(y))/20000,y(20:end), "r-"), grid on
xlabel("Tiempo [s]")
ylabel("Voltaje [mV]")
yline(2600, "Label","Voltaje pico", "LineStyle","--","LabelHorizontalAlignment","left")
xlim([0,2.6])