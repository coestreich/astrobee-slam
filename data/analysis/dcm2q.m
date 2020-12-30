function q = dcm2q(dcm)
    
    tr = dcm(1,1) + dcm(2,2) + dcm(3,3);
    if(tr > 0)
        S = sqrt(tr + 1) * 2;
        qw = 0.25 * S;
        qx = (dcm(3,2) - dcm(2,3)) / S;
        qy = (dcm(1,3) - dcm(3,1)) / S;
        qz = (dcm(2,1) - dcm(1,2)) / S;
    elseif((dcm(1,1) > dcm(2,2)) && (dcm(1,1) > dcm(3,3)))
        S = sqrt(1.0 + dcm(1,1) - dcm(2,2) - dcm(3,3))*2;
        qw = (dcm(3,2) - dcm(2,3))/S;
        qx = 0.25*S;
        qy = (dcm(1,2) + dcm(2,1))/S;
        qz = (dcm(1,3) + dcm(3,1))/S;
    elseif(dcm(2,2) > dcm(3,3))
        S = sqrt(1.0 + dcm(2,2) - dcm(1,1) - dcm(3,3))*2;
        qw = (dcm(1,3) - dcm(3,1))/S;
        qx = (dcm(1,2) + dcm(2,1))/S;
        qy = 0.25*S;
        qz = (dcm(2,3) + dcm(3,2))/S;
    else
        S = sqrt(1.0 + dcm(3,3) - dcm(1,1) - dcm(2,2))*2;
        qw = (dcm(2,1) - dcm(1,2))/S;
        qx = (dcm(1,3) + dcm(3,1))/S;
        qy = (dcm(2,3) + dcm(3,2))/S;
        qz = 0.25*S;
    end
    q = [qx;qy;qz;qw]';

end

