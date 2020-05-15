Klut = zeros(10,13,2);
Eqlut = zeros(10,13,2);
for j = 1:10        % angolo
    for i = 1:13    % velocità
         par = LinPlant(j*3,i*10);
         Klut(j,i,:) = par(:,1);
         Eqlut(j,i,:) = par(:,2);
    end
end