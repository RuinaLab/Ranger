%Example use of plotField and parseLog
close all; clear all; clc

%Organize everything from the log file.
data = parseLog('../../ROBO_DAQ_7/Test-20150927-163251.txt');

%Get all the variable names {which are the names of the fields of the
%super-struct.
names = fieldnames(data);
plotField(data,{names{1},names{2},names{3},names{4},names{5}},'subplots')

