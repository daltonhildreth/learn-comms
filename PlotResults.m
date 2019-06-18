close all
clear all
clc

o = load('data/var_a.csv');
o_x = o(:,1); % 1st column
o_cttc = o(:,5-3:3:end); % every 2nd
o_norm = o(:,6-3:3:end); % ever 3rd
o_ttc = o(:,7-3:3:end); % every 4th

o_cttc_m = mean(o_cttc, 2); % per row
o_cttc_sm = movmean(o_cttc_m, 11);
o_cttc_s = std(o_cttc, 0, 2);
o_cttc_ss = movmean(o_cttc_s, 11);
o_cttc_u = o_cttc_sm + o_cttc_ss;%prctile(o_cttc, 90, 2);
o_cttc_l = o_cttc_sm - o_cttc_ss;%prctile(o_cttc, 10, 2);

o_ttc_m = mean(o_ttc,2); % per row
o_ttc_sm = movmean(o_ttc_m, 11);
o_ttc_s = std(o_ttc, 0, 2);
o_ttc_ss = movmean(o_ttc_s, 11);
o_ttc_u = o_ttc_sm + o_ttc_ss;%prctile(o_ttc, 90, 2);
o_ttc_l = o_ttc_sm - o_ttc_ss;%prctile(o_ttc, 10, 2); 


o_norm_m = mean(o_norm, 2); % per row
o_norm_sm = movmean(o_norm_m, 11);
o_norm_s = std(o_norm, 0, 2);
o_norm_ss = movmean(o_norm_s, 11);
o_norm_u = o_norm_sm + o_norm_ss;
o_norm_l = o_norm_sm - o_norm_ss;


figure;
ax1 = gca;
hold on;
fill([o_x; flipud(o_x)], [o_cttc_l; flipud(o_cttc_u)], 'b', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
cm = plot(o_x, o_cttc_m, 'b', 'LineWidth', 1); cm.Color(4) = 0.5;
plot(o_x, o_cttc_sm, 'b', 'LineWidth', 2);
fill([o_x; flipud(o_x)], [o_ttc_l; flipud(o_ttc_u)], 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
tm = plot(o_x, o_ttc_m, 'r', 'LineWidth', 1); tm.Color(4) = 0.5;
plot(o_x, o_ttc_sm, 'r', 'LineWidth', 2);
line([40 40], [0 160], 'Color', 'k');
xlim([5,100]);
xlabel('Number of Agents');
ylabel('Overhead (s)');
ax1.FontSize = 14;
outerpos = ax1.OuterPosition;
ti = ax1.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax1.Position = [left bottom ax_width ax_height];
hold off;


ax2 = axes('Position',[.18,.5,.2,.4]);
box on 
hold on;
% fill([o_x; flipud(o_x)], [o_norm_l; flipud(o_norm_u)], 'm', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
on = plot(o_x, o_norm_m, 'm', 'LineWidth', 1); on.Color(4) = 0.5;
plot(o_x, o_norm_sm, 'm', 'LineWidth', 2);
line([40 40], [0.23 1], 'Color', 'k');
set(gca,'YScale','log')
xlim([17,100]);
%ylabel('Normalized Overhead');
hold off;