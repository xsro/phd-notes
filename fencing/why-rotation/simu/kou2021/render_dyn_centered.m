colors=lines(p.N);

%%
figure();
for ia=1:p.N
    nexttile;
    x=p.x(1:end,p.idx(1,ia))-p.x(1:end,end-1);
    y=p.x(1:end,p.idx(2,ia))-p.x(1:end,end);
    plot(x,y,'-',"Color",colors(ia,:));
    axis equal
end

%%
figure;hold on;
for ia=1:p.N
    x=p.x(1:end,p.idx(1,ia))-p.x(1:end,end-1);
    y=p.x(1:end,p.idx(2,ia))-p.x(1:end,end);
    plot(x,y,'-',"Color",colors(ia,:));
    axis equal
end

it=1;
h_target=plot(0,0,"kv");
handle_agents=repmat(h_target,p.N,1);
for ia=1:p.N
    xi=p.x(it,p.idx(1:2,ia))-p.x(it,end-1:end);
    handle_agents(ia)=plot(xi(1),xi(2), ...
        'o','MarkerFaceColor',colors(ia,:) ...
        ,"Color",colors(ia,:), ...
        'MarkerSize',10);
end
axis equal;
%%
for it=1:100:length(p.t)
    for ia=1:p.N
        xi=p.x(it,p.idx(1:2,ia))-p.x(it,end-1:end);
        set(handle_agents(ia),'XData',xi(1),'YData',xi(2));
    end
    title(sprintf("t=%.2f",p.t(it)));
    pause(0.1);
    exportgraphics(gcf,"center.gif","Append",it>1)
end