function eul = dcm2eul313(dcm)

    eul1 = atan2(dcm(3,1), dcm(3,2));
    eul2 = acos(dcm(3,3)); 
    eul3 = -atan2(dcm(1,3), dcm(2,3));
    
    eul = [eul1, eul2, eul3];

end

