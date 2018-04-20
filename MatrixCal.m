clc;
clear;
syms a00 a01 a02 a10 a11 a12 a20 a21 a22 g
A=[a00 a01 a02;a10 a11 a12;a20 a21 a22];
G=[g 0 0;0 g 0;0 0 g];
inv(A)*G
