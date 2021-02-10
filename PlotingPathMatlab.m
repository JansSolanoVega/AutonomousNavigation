clear all;
clc;
delete(instrfind({'Port'},{'COM3'}));
puerto_serial=serial('COM3','Terminator','CR/LF');
puerto_serial.Baudrate=9600;
warning('off','MATLAB:serial:fscanf:unsuccessfulread');

fopen(puerto_serial);

counter_samples=1;

hold on;
plot(50,50,'*g')
while counter_samples<5000

  data=fscanf(puerto_serial,'%.2f %.2f');
    C=strsplit(data,',');
    x=str2num(C{1});    
    y=str2num(C{2});
    plot(x,y,'*')
    pause(0.01);
    hold on
    counter_samples=counter_samples+1;
end 

fclose(puerto_serial);



