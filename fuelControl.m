function [engSpd, engTrq] = fuelControl(engState, vehSpd, vehAcc, veh)



[dmdSpd, dmdTrq, vehPrf] = hev_drivetrain(vehSpd, vehAcc, veh);



% ENGINE
max_Spd_eng = veh.eng.maxSpd;
min_Spd_eng = veh.eng.idleSpd;


eng_speeds = linspace(min_Spd_eng, max_Spd_eng, 100);
opt_Torques_eng = veh.eng.oolTrq(eng_speeds);
opt_Powers_eng = eng_speeds.*opt_Torques_eng;

vector_bsfc = veh.eng.bsfcMap(eng_speeds, opt_Torques_eng);
[min_bsfc, i] = min(vector_bsfc);
spd_opt_eng = eng_speeds(i);
trq_opt_eng = veh.eng.oolTrq(spd_opt_eng);
opt_Pwr_eng = spd_opt_eng.*trq_opt_eng;

[maxpower, index] = max(opt_Powers_eng);
speed_max_Pwr = eng_speeds(index);
torque_max_Pwr = veh.eng.maxTrq(speed_max_Pwr);
max_Pwr_eng = speed_max_Pwr.*torque_max_Pwr;



effMot = veh.mot.effMap(dmdSpd, dmdTrq);

if dmdTrq >= 0
    demPwr = dmdTrq*dmdSpd/effMot;
else
    demPwr = dmdTrq*dmdSpd*effMot;
end



% % ELECTRIC MOTOR
% max_Trq_mot = veh.mot.maxTrq(dmdSpd);
% min_Trq_mot = veh.mot.minTrq(dmdSpd);
% max_Spd_mot = veh.mot.maxSpd;
% max_Pwr_mot = veh.mot.maxPwr;
% 


% % GENERATOR
% effGen = 0.9;
% max_Spd_gen = veh.gen.maxSpd;
% max_Pwr_gen = veh.gen.maxPwr;
% max_Trq_gen = veh.gen.maxTrq(dmdSpd);
% min_Trq_gen = veh.gen.minTrq(dmdSpd);


% % BATTERY
% max_Curr_batt = veh.batt.maxCurr;
% min_Curr_batt = veh.batt.minCurr;
% 
% max_Pwr_batt = veh.batt.maxPwr(SOC);
% min_Pwr_batt = veh.batt.minPwr(SOC);







if engState == 1        

    %when engine is turned on, we check how much is the demanded power and
    %we compare it with some intervals, which will allow us to better
    %regulate the P_eng_cmd

    if abs(demPwr) <=  abs(opt_Pwr_eng)     %if the demanded power is lower or equal than the optimal, we can use it and get the best performances
        P_eng_cmd = opt_Pwr_eng;

    elseif abs(demPwr) <=  1.25*abs(opt_Pwr_eng)    %if the power demand is inside a range of 25% over the optimal power value, we implement a gain
        gain = (1 + (abs(demPwr)- abs(opt_Pwr_eng))/abs(opt_Pwr_eng));      %this gain is proportional to the difference between the demPwr and optimal one
        P_eng_cmd = gain * opt_Pwr_eng;     %thanks to this gain we can now move slightly around the optimal value instead of running at max power, saving fuel.

    elseif abs(demPwr)>1.25*abs(opt_Pwr_eng) && abs(demPwr) <= 1.75*abs(opt_Pwr_eng)    %same procedure is applied in a new range, now between 25-75% over the optimal power value.
        gain = 1.1*(1 - (abs(max_Pwr_eng)-abs(demPwr))/abs(max_Pwr_eng));       %differently from before we now make the gain proportional to the distance between the demPwr and the maximum one. %Thanks to our simulation we have seen that a boost of 10% is ideal to save fuel and charge the battery
        P_eng_cmd = gain*max_Pwr_eng;       %as before the gain is multiplying the maxPwr, providing more power than the optimal, but still not running full gas.

    else                                                                                %in all the other cases for which we need more power we also use a gain to regulate the maximum value
        gain = (1 - (abs(max_Pwr_eng)-abs(demPwr))/abs(max_Pwr_eng));       %this gain has now no boost, as there are very few occasion in which we need it and so we can avoid to loose efficiency
        P_eng_cmd = gain*max_Pwr_eng;

        if P_eng_cmd >= max_Pwr_eng     %an extra check is now implemented, to avoid that our controls overcome the limits of the engine power
            P_eng_cmd = max_Pwr_eng;
        end

    end


    engSpd = veh.eng.oolSpd(P_eng_cmd);
    engTrq = P_eng_cmd/engSpd;
    
end




engSpd = max(engSpd, veh.eng.idleSpd);
engTrq = P_eng_cmd/engSpd;
end
