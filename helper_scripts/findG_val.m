% Helper Functions:
% finding gripper value using object width
function g_val = findG_val(obj_width, label) 
    if (label == 3) % pouches
        g_val = 0.53; 
    elseif (label == 2) % can's
        g_val = 0.228;
        if (obj_width > 0.0595)
            g_val = 0.230;
        end
    elseif (label == 1 && obj_width < 0.02)
        g_val = 0.68;
    elseif (label == 1) % bottles
        if (obj_width > 0.0550) % bottle lying down
            g_val = 0.209;
        else
            g_val = 0.516;
        end
    end
end