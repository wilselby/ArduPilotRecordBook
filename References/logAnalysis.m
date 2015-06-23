

close all;
clear all;
clc;

load 2013-06-30_13-23-14_WAP_SURVEY.tlog.mat

figure;
hold on;
subplot(1,3,1);
plot(alt_mavlink_vfr_hud_t(:,2))
subplot(1,3,2);
plot(airspeed_mavlink_vfr_hud_t(:,2))
subplot(1,3,3);
plot(throttle_mavlink_vfr_hud_t(:,2))