clc
clear
close all
%%

data = readtable("datos.xlsx")

%%


plot(data.Tiempo,data.Salida), hold on, grid on
plot(data.Tiempo,data.Entrada)
xlabel("Time [s]");
ylabel("Value [mag]");


%%
% Trabajando de 10-20%
plot(data.Tiempo,data.Salida), hold on, grid on
plot(data.Tiempo,data.Entrada)
xlim([190,310])
xlabel("Time [s]");
ylabel("Value [mag]");

ident_output = data.Salida(190:310)
ident_input = data.Entrada(190:310)

%%
gain = (data.Salida(280) - data.Salida(190))/10.;  
sett_time = 40; % 50secs
tau = sett_time/4;

%No encontramos retraso, asi tenemos una ft descrita de la siguiente
%manera:

s = tf("s");

G_ol = gain/(tau*s + 1);

G_cl = feedback(G_ol, 1);

bode(G_ol);
% En principio, no existe ganancia ultima

%%

fact_amort = 0.75;
tau_cl = 0.6*tau;






