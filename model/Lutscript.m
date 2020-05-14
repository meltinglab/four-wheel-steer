Klut = eye(30,13)*0;
Eqlut = eye(30,13)*0;
for j = 1:10        % angolo
    for i = 1:13    % velocità
         par = LinPlant(j*3,i*10);
         Klut(((j-1)*3+1):((j-1)*3+3),i) = par(:,1);
         Eqlut(((j-1)*3+1):((j-1)*3+3),i) = par(:,2);
    end
end