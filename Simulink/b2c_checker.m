function [Iout,Qout] = b2c_checker(Iin,Qin)
% Rearranging the eigenvectors and associated eigenvalues

[~,ix]= max(diag(Iin));
[~,iz]= min(diag(Iin));
iy = setdiff([1 2 3],sort([ix,iz]));

Iout = diag([Iin(ix,ix),Iin(iy,iy),Iin(iz,iz)]);

if norm(double(Qin(:,ix)<0)) > 1
    Qin(:,ix) = -Qin(:,ix);
end

if norm(double(Qin(:,iz)<0)) > 1
    Qin(:,iz) = -Qin(:,iz);
end

Qout = [Qin(:,ix),cross(Qin(:,iz),Qin(:,ix)),Qin(:,iz)];

end

