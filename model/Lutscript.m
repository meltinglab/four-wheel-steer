lut = eye(20,13)*0;
for j = 1:10        % angolo
    for i = 1:13    % velocità
        lut(((j-1)*2+1):((j-1)*2+2),i) = LinPlant(j*3,i*10);
    end
end
