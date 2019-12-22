function [Nrich,Nrich_pct,fbest,tp_best,tp_rich] = EvaluateAnatomiesRichnessForBestInertiaStructure(fref,tai_step,wi,xi_ai,xi_pi,structure2check,gsCoMi0,Mbi)
% Nrich is given as absolute and percetage of the best anatomies inside a structure
% fbest s the best f=maxDCI+minDCI/2 index values of all possible anatomies
% tp_best is the best anatomy of structure;
% tp_rich is a nx4 array of all antomies better than reference

fbest = 1000000;        % trash value for best DCI anatomy of structure    

tpi_step = deg2rad(30);
Nrich = 0;              % counter of anatomies with fanat>fref
p = 0;                  % structure's anatomies global counter

switch structure2check
    case '0110110'
        p1 = 0;
        for tp1=-1.5708:tpi_step:1.5708
            p1 = p1 +1; p2 = 0;
            for tp2=-1.5708:tpi_step:1.5708
                p2 = p2 +1; p3 = 0;
                for tp3=-1.5708:tpi_step:1.5708
                    p3 = p3 +1; p4 = 0;
                    for tp4=-1.5708:tpi_step:1.5708
                        p = p+1;
                        p4 = p4 +1; 
                        % Compute 
                        P1 = twistexp(xi_pi(:,1),tp1)*twistexp(xi_pi(:,2),tp2);
                        P2 = twistexp(xi_pi(:,3),tp3)*twistexp(xi_pi(:,4),tp4);
                        Pi(:,:,1) = P1;
                        Pi(:,:,2) = P2;
                        % Search in C-Space
                        fanat(p1,p2,p3,p4) = ActiveCspace_for_AnatomyRichnessCalculation(tai_step,wi,xi_ai,Pi,gsCoMi0,Mbi);
                        
                        % checks if anatomy is better than reference
                        if fanat(p1,p2,p3,p4)<fref
                            Nrich = Nrich+1;
                            tp_rich(Nrich,:) = [tp1 tp2 tp3 tp4];
                        end
                        % finds the best anatomy of structure
                        if fanat(p1,p2,p3,p4)<fbest
                            fbest = fanat(p1,p2,p3,p4);
                            tp_best = [tp1 tp2 tp3 tp4];
                        end                        
                        
                    end
                end
            end
        end
        
        Nrich_pct = Nrich/p;
        
    case '01010'
        
        p1 = 0;
        for tp1=-1.5708:tpi_step:1.5708
            p1 = p1 +1; p2 = 0;
            for tp2=-1.5708:tpi_step:1.5708
                p = p+1;
                p2 = p2 +1; 
                % Compute 
                P1 = twistexp(xi_pi(:,1),tp1);
                P2 = twistexp(xi_pi(:,2),tp2);
                Pi(:,:,1) = P1;
                Pi(:,:,2) = P2;
                % Search in C-Space
                fanat(p1,p2) = ActiveCspace_for_AnatomyRichnessCalculation(tai_step,wi,xi_ai,Pi,gsCoMi0,Mbi);

                % checks if anatomy is better than reference
                if fanat(p1,p2)<fref
                    Nrich = Nrich+1;
                    tp_rich(Nrich,:) = [tp1 tp2];
                end
                % finds the best anatomy of structure
                if fanat(p1,p2)<fbest
                    fbest = fanat(p1,p2);
                    tp_best = [tp1 tp2];
                end  
                        
            end
        end
        
        Nrich_pct = Nrich/p;
        
        
    case '010110'
        
        p1 = 0;
        for tp1=-1.5708:tpi_step:1.5708
            p1 = p1 +1; p2 = 0;
            for tp2=-1.5708:tpi_step:1.5708
                p2 = p2 +1; p3 = 0;
                for tp3=-1.5708:tpi_step:1.5708
                    p = p+1;
                    p3 = p3 +1; 

                    % Compute 
                    P1 = twistexp(xi_pi(:,1),tp1);
                    P2 = twistexp(xi_pi(:,2),tp2)*twistexp(xi_pi(:,3),tp3);
                    Pi(:,:,1) = P1;
                    Pi(:,:,2) = P2;
                    % Search in C-Space
                    fanat(p1,p2,p3) = ActiveCspace_for_AnatomyRichnessCalculation(tai_step,wi,xi_ai,Pi,gsCoMi0,Mbi);
                    
                    % checks if anatomy is better than reference
                    if fanat(p1,p2,p3)<fref
                        Nrich = Nrich+1;
                        tp_rich(Nrich,:) = [tp1 tp2 tp3];
                    end
                    % finds the best anatomy of structure
                    if fanat(p1,p2,p3)<fbest
                        fbest = fanat(p1,p2,p3);
                        tp_best = [tp1 tp2 tp3];
                    end  
                        
                end
            end
        end
        
        Nrich_pct = Nrich/p;
        
        
    case '011010'
        
        p1 = 0;
        for tp1=-1.5708:tpi_step:1.5708
            p1 = p1 +1; p2 = 0;
            for tp2=-1.5708:tpi_step:1.5708
                p2 = p2 +1; p3 = 0;
                for tp3=-1.5708:tpi_step:1.5708
                    p = p+1;
                    p3 = p3 +1; 

                    % Compute 
                    P1 = twistexp(xi_pi(:,1),tp1)*twistexp(xi_pi(:,2),tp2);
                    P2 = twistexp(xi_pi(:,3),tp3);
                    Pi(:,:,1) = P1;
                    Pi(:,:,2) = P2;
                    % Search in C-Space
                    fanat(p1,p2,p3) = ActiveCspace_for_AnatomyRichnessCalculation(tai_step,wi,xi_ai,Pi,gsCoMi0,Mbi);
                    
                    % checks if anatomy is better than reference
                    if fanat(p1,p2,p3)<fref
                        Nrich = Nrich+1;
                        tp_rich(Nrich,:) = [tp1 tp2 tp3];
                    end
                    % finds the best anatomy of structure
                    if fanat(p1,p2,p3)<fbest
                        fbest = fanat(p1,p2,p3);
                        tp_best = [tp1 tp2 tp3];
                    end  
                        
                end
            end
        end
        
        Nrich_pct = Nrich/p;

    otherwise
        disp('Structure given is not accepted.')
        check = 'error';  
end   

end