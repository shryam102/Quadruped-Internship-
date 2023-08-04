%Function to evaluate jacobian matrix of individual leg

function array = Jacobian_plot(leg,L1,L2,L3,Th_init)
         
         array = [                                                                                                                                                               0,                                                   L3*sin(Th_init(2))*sin(Th_init(3)) - L3*cos(Th_init(2))*cos(Th_init(3)) - L2*cos(Th_init(2)),                                   L3*sin(Th_init(2))*sin(Th_init(3)) - L3*cos(Th_init(2))*cos(Th_init(3));
         L1*cos(Th_init(1)) + L2*cos(Th_init(1))*cos(Th_init(2)) + L3*cos(Th_init(1))*cos(Th_init(2))*cos(Th_init(3)) - L3*cos(Th_init(1))*sin(Th_init(2))*sin(Th_init(3)), - L2*sin(Th_init(1))*sin(Th_init(2)) - L3*cos(Th_init(2))*sin(Th_init(1))*sin(Th_init(3)) - L3*cos(Th_init(3))*sin(Th_init(1))*sin(Th_init(2)), - L3*cos(Th_init(2))*sin(Th_init(1))*sin(Th_init(3)) - L3*cos(Th_init(3))*sin(Th_init(1))*sin(Th_init(2));
         L1*sin(Th_init(1)) + L2*cos(Th_init(2))*sin(Th_init(1)) + L3*cos(Th_init(2))*cos(Th_init(3))*sin(Th_init(1)) - L3*sin(Th_init(1))*sin(Th_init(2))*sin(Th_init(3)),   L2*cos(Th_init(1))*sin(Th_init(2)) + L3*cos(Th_init(1))*cos(Th_init(2))*sin(Th_init(3)) + L3*cos(Th_init(1))*cos(Th_init(3))*sin(Th_init(2)),   L3*cos(Th_init(1))*cos(Th_init(2))*sin(Th_init(3)) + L3*cos(Th_init(1))*cos(Th_init(3))*sin(Th_init(2));];
         
         if leg=="LF"
             qLF_HAA = Th_init(1);
             qLF_HFE = Th_init(2);
             qLF_KFE = Th_init(3);
             array = [                                                                                                                                    0,                                          L3*sin(qLF_HFE)*sin(qLF_KFE) - L3*cos(qLF_HFE)*cos(qLF_KFE) - L2*cos(qLF_HFE),                             L3*sin(qLF_HFE)*sin(qLF_KFE) - L3*cos(qLF_HFE)*cos(qLF_KFE);
                     L1*cos(qLF_HAA) + L2*cos(qLF_HAA)*cos(qLF_HFE) + L3*cos(qLF_HAA)*cos(qLF_HFE)*cos(qLF_KFE) - L3*cos(qLF_HAA)*sin(qLF_HFE)*sin(qLF_KFE), - L2*sin(qLF_HAA)*sin(qLF_HFE) - L3*cos(qLF_HFE)*sin(qLF_HAA)*sin(qLF_KFE) - L3*cos(qLF_KFE)*sin(qLF_HAA)*sin(qLF_HFE), - L3*cos(qLF_HFE)*sin(qLF_HAA)*sin(qLF_KFE) - L3*cos(qLF_KFE)*sin(qLF_HAA)*sin(qLF_HFE);
                     L1*sin(qLF_HAA) + L2*cos(qLF_HFE)*sin(qLF_HAA) + L3*cos(qLF_HFE)*cos(qLF_KFE)*sin(qLF_HAA) - L3*sin(qLF_HAA)*sin(qLF_HFE)*sin(qLF_KFE),   L2*cos(qLF_HAA)*sin(qLF_HFE) + L3*cos(qLF_HAA)*cos(qLF_HFE)*sin(qLF_KFE) + L3*cos(qLF_HAA)*cos(qLF_KFE)*sin(qLF_HFE),   L3*cos(qLF_HAA)*cos(qLF_HFE)*sin(qLF_KFE) + L3*cos(qLF_HAA)*cos(qLF_KFE)*sin(qLF_HFE);];
         

         elseif leg =="LH"
             qLH_HAA = Th_init(1);
             qLH_HFE = Th_init(2);
             qLH_KFE = Th_init(3);

             array = [                                                                                                                                    0,                                         L3*sin(qLH_HFE)*sin(qLH_KFE) - L3*cos(qLH_HFE)*cos(qLH_KFE) - L2*cos(qLH_HFE),                              L3*sin(qLH_HFE)*sin(qLH_KFE) - L3*cos(qLH_HFE)*cos(qLH_KFE);
                     L1*cos(qLH_HAA) + L2*cos(qLH_HAA)*cos(qLH_HFE) + L3*cos(qLH_HAA)*cos(qLH_HFE)*cos(qLH_KFE) - L3*cos(qLH_HAA)*sin(qLH_HFE)*sin(qLH_KFE), - L2*sin(qLH_HAA)*sin(qLH_HFE) - L3*cos(qLH_HFE)*sin(qLH_HAA)*sin(qLH_KFE) - L3*cos(qLH_KFE)*sin(qLH_HAA)*sin(qLH_HFE), - L3*cos(qLH_HFE)*sin(qLH_HAA)*sin(qLH_KFE) - L3*cos(qLH_KFE)*sin(qLH_HAA)*sin(qLH_HFE);
                     L1*sin(qLH_HAA) + L2*cos(qLH_HFE)*sin(qLH_HAA) + L3*cos(qLH_HFE)*cos(qLH_KFE)*sin(qLH_HAA) - L3*sin(qLH_HAA)*sin(qLH_HFE)*sin(qLH_KFE),   L2*cos(qLH_HAA)*sin(qLH_HFE) + L3*cos(qLH_HAA)*cos(qLH_HFE)*sin(qLH_KFE) + L3*cos(qLH_HAA)*cos(qLH_KFE)*sin(qLH_HFE),   L3*cos(qLH_HAA)*cos(qLH_HFE)*sin(qLH_KFE) + L3*cos(qLH_HAA)*cos(qLH_KFE)*sin(qLH_HFE);];

         elseif leg == "RF"
             qRF_HAA = Th_init(1);
             qRF_HFE = Th_init(2);
             qRF_KFE = Th_init(3);

             array = [                                                                                                                                    0,                                          L2*cos(qRF_HFE) + L3*cos(qRF_HFE)*cos(qRF_KFE) - L3*sin(qRF_HFE)*sin(qRF_KFE),                             L3*cos(qRF_HFE)*cos(qRF_KFE) - L3*sin(qRF_HFE)*sin(qRF_KFE);
                     L1*cos(qRF_HAA) + L2*cos(qRF_HAA)*cos(qRF_HFE) + L3*cos(qRF_HAA)*cos(qRF_HFE)*cos(qRF_KFE) - L3*cos(qRF_HAA)*sin(qRF_HFE)*sin(qRF_KFE), - L2*sin(qRF_HAA)*sin(qRF_HFE) - L3*cos(qRF_HFE)*sin(qRF_HAA)*sin(qRF_KFE) - L3*cos(qRF_KFE)*sin(qRF_HAA)*sin(qRF_HFE), - L3*cos(qRF_HFE)*sin(qRF_HAA)*sin(qRF_KFE) - L3*cos(qRF_KFE)*sin(qRF_HAA)*sin(qRF_HFE);
                     L1*sin(qRF_HAA) + L2*cos(qRF_HFE)*sin(qRF_HAA) + L3*cos(qRF_HFE)*cos(qRF_KFE)*sin(qRF_HAA) - L3*sin(qRF_HAA)*sin(qRF_HFE)*sin(qRF_KFE),   L2*cos(qRF_HAA)*sin(qRF_HFE) + L3*cos(qRF_HAA)*cos(qRF_HFE)*sin(qRF_KFE) + L3*cos(qRF_HAA)*cos(qRF_KFE)*sin(qRF_HFE),   L3*cos(qRF_HAA)*cos(qRF_HFE)*sin(qRF_KFE) + L3*cos(qRF_HAA)*cos(qRF_KFE)*sin(qRF_HFE);];

         elseif leg =="RH"
             qRH_HAA = Th_init(1);
             qRH_HFE = Th_init(2);
             qRH_KFE = Th_init(3);

             array = [                                                                                                                                    0,                                          L2*cos(qRH_HFE) + L3*cos(qRH_HFE)*cos(qRH_KFE) - L3*sin(qRH_HFE)*sin(qRH_KFE),                             L3*cos(qRH_HFE)*cos(qRH_KFE) - L3*sin(qRH_HFE)*sin(qRH_KFE);
                     L1*cos(qRH_HAA) + L2*cos(qRH_HAA)*cos(qRH_HFE) + L3*cos(qRH_HAA)*cos(qRH_HFE)*cos(qRH_KFE) - L3*cos(qRH_HAA)*sin(qRH_HFE)*sin(qRH_KFE), - L2*sin(qRH_HAA)*sin(qRH_HFE) - L3*cos(qRH_HFE)*sin(qRH_HAA)*sin(qRH_KFE) - L3*cos(qRH_KFE)*sin(qRH_HAA)*sin(qRH_HFE), - L3*cos(qRH_HFE)*sin(qRH_HAA)*sin(qRH_KFE) - L3*cos(qRH_KFE)*sin(qRH_HAA)*sin(qRH_HFE);
                     L1*sin(qRH_HAA) + L2*cos(qRH_HFE)*sin(qRH_HAA) + L3*cos(qRH_HFE)*cos(qRH_KFE)*sin(qRH_HAA) - L3*sin(qRH_HAA)*sin(qRH_HFE)*sin(qRH_KFE),   L2*cos(qRH_HAA)*sin(qRH_HFE) + L3*cos(qRH_HAA)*cos(qRH_HFE)*sin(qRH_KFE) + L3*cos(qRH_HAA)*cos(qRH_KFE)*sin(qRH_HFE),   L3*cos(qRH_HAA)*cos(qRH_HFE)*sin(qRH_KFE) + L3*cos(qRH_HAA)*cos(qRH_KFE)*sin(qRH_HFE);];

         end
end
