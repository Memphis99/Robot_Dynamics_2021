function R = quatToRotMat(q)
  % Input: quaternion [w x y z]
  % Output: corresponding rotation matrix
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  R = [q(0)^2+q(1)^2-q(2)^2-q(3)^2, 2*q(1)*q(2)-2*q(0)*q(3), 2*q(0)*q(2)+2*q(1)*q(3);
       2*q(0)*q(3)+2*q(1)*q(2), q(0)^2-q(1)^2+q(2)^2-q(3)^2, 2*q(2)*q(3)-2*q(0)*q(1);
       2*q(1)*q(3)-2*q(0)*q(2), 2*q(0)*q(1)+2*q(2)*q(3), q(0)^2-q(1)^2-q(2)^2+q(3)^2];
end
