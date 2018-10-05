% function exportToPdf(fig, width, height, file)
%    fig      figure handle
%    width    width in centimeters
%    height   height in centimeters
%    file     file-path as string
%
%  example: exportToPdf(gcf, 5, 5, [name '.pdf']);
%
%  Markus Enzweiler, uni-heidelberg.enzweiler@daimler.com
%  2009-10-20



function exportToPdf(fig, width, height, file)


set(fig, 'Units', 'centimeters');
set(fig, 'PaperUnits', 'centimeters');

pos = get(fig, 'Position');

set(fig, 'Position', [pos(1) pos(1) width height]);

set(fig, 'PaperSize', [ width height]);
set(fig, 'PaperPosition', [0 0  width height]);

print( gcf, '-dpdf', file);



