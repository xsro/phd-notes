figure();
hold on;
it=1;
h_target=plot(p.x(it,end-1),p.x(it,end),'kv');
handle_agents=repmat(h_target,p.N,1);
for ia=1:p.N
    plot(p.x(:,p.idx(1,ia)),p.x(:,p.idx(2,ia)),'k-');
    xi=p.x(it,p.idx(1:2,ia));
    handle_agents(ia)=plot(xi(1),xi(2),'ro','MarkerFaceColor','red','MarkerSize',10);
end
axis equal;


%%
for it=1:10:length(p.t)
    set(h_target,'XData',p.x(it,end-1),'YData',p.x(it,end));
    for ia=1:p.N
        xi=p.x(it,p.idx(1:2,ia));
        set(handle_agents(ia),'XData',xi(1),'YData',xi(2));
    end
    title(sprintf("t=%.2f",p.t(it)));
    pause(0.1);
end