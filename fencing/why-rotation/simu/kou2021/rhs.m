function [dxdt,s] = rhs(t,x,p)
    dxdt=zeros(length(x),1);
    s=struct();
    s.debug_p1=zeros(2,1);
    s.debug_pv=zeros(2,1);
    s.debug_pvn=zeros(p.N,1);
    s.debug_pv2=zeros(p.N,1);
    s.debug_pi0phi=zeros(p.N,1);
    x0=x(end-1:end);
    for ia=1:p.N
        xi=x(p.idx(1:2,ia));
        vi=x(p.idx(3:4,ia));

        s.debug_p1=s.debug_p1+(xi-x0);

        repulse=zeros(2,1);
        for ib=1:p.N
            if ia==ib
                continue;
            end
            xj=x(p.idx(1:2,ib));
            dij=norm(xi-xj);
            alpha=collision_avoidance(dij,1000,p.d,p.mu);
            repulse=repulse+alpha*(xi-xj)/dij;
        end

        dxdt(p.idx(1:2,ia))=p.k1*(x0-xi)+vi+repulse;
        dxdt(p.idx(3:4,ia))=p.k2*(x0-xi);

        agent_velocity=dxdt(p.idx(1:2,ia))-p.targetv;
        s.debug_pv=s.debug_pv+agent_velocity;
        R=[cos(pi/2),-sin(pi/2);sin(pi/2),cos(pi/2)];
        pi0unit=R*(xi-x0)/norm(x0-xi);
        s.debug_pv2(ia)=dot(agent_velocity/norm(x0-xi),pi0unit);
        s.debug_pvn(ia)=norm(agent_velocity);
        s.debug_pi0phi(ia)=(xi-x0)'*R*repulse;
    end
    s.debug_p1=s.debug_p1/p.N;
    s.debug_pv=s.debug_pv/p.N;

    dxdt(end-1:end)=p.targetv;
end