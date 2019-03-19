function A = compute_dh_matrix(r, alpha, d, theta)
% -This calculates the DH matrix using the DH parameters (all symbols are
% standard)

    A = eye(4);
    ct=cos(theta);
    st=sin(theta);
    ca=cos(alpha);
    sa=sin(alpha);
    A=[ct, -st*ca, st*sa, r*ct;st, ct*ca, -ct*sa, r*st; 0,sa,ca,d;0, 0, 0, 1];    
end