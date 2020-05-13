Klut = zeros(10,13,2);
%Klut = eye(20,13)*0;
for i = 1:10        % angolo
    for j = 1:13    % velocità
        %Klut(((j-1)*2+1):((j-1)*2+2),i) = LinPlant(j*3,i*10);
        Klut(i,j,:) = LinPlant(i*3,j*10);
    end
end
