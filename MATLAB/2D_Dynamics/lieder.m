function Lgh=lieder(g,h,X)
% -This function outputs the lie derivative of 'h' along function 'g' wrt
%  state X
%  both inputs must be column vectors all symbolic with independent variable t

syms t

h_l=h(t); %getting the function handle 'h'
X_l=X(t);

fd=@functionalDerivative;

jachx=sym(zeros(length(h_l),length(X_l)));

for i=1:length(h_l)
    for j=1:length(X_l);
        jachx(i,j)=functionalDerivative(h_l(i),X_l(j)); %functional derivative is required as 'h_l' and 'X_l' are functions of independent variable 't'
    end
end

Lgh=jachx*g;

end
