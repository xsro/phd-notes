function export_pdf(fig, fname, width_in)
% EXPORT_PDF  将图以矢量 PDF 导出 (论文插图统一格式)
%
%   export_pdf(fig, fname)         宽度取屏幕尺寸 (Position/96), 保持现有观感
%   export_pdf(fig, fname, width_in)  指定物理宽度 [in] (用于按版面宽度设计的新图)
%
% 说明: PDF 为矢量输出, 字体/线条不随 LaTeX 缩放而失真; 图内字号在版面
% 尺寸等于 width_in 时即为创建时的 pt 值。3D 轴使用 painters 渲染器,
% 本项目插图不含透明度, 可保持矢量。

    if nargin < 3 || isempty(width_in)
        pos = get(fig, 'Position');
        width_in = pos(3)/96;
    end
    pos = get(fig, 'Position');
    height_in = width_in * pos(4)/pos(3);

    set(fig, 'PaperUnits', 'inches', ...
        'PaperPosition', [0 0 width_in height_in], ...
        'PaperSize',     [width_in height_in]);
    print(fig, '-dpdf', '-painters', '-r600', fname);
    fprintf('图已保存: %s\n', fname);
end
