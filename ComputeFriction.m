function [f_friction]=ComputeFriction(c,f_current,vt)

    f_friction=-c*norm(f_current)*vt;

end


