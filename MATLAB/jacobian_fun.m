function list = jacobian_fun(T,Q)
         matrix = sym('A',[3,length(Q)]);
         for i =1:3
             for j=1:length(Q)
                 matrix(i,j) = diff(T(i),Q(j));
             end
         end
         list = matrix;
end