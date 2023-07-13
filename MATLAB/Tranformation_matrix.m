%Function to evaluate the Tranformation Matrix with given Rotation matrix and Postition Vector 

function x = Tranformation_matrix(R,Q)
         A = sym('A',[4,4]);
         A(1:3,1:3) = R;
         A(1:3,4) = Q;
         A(4,:) = [0 0 0 1];
         x = A;
end