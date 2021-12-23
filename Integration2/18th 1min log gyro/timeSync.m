function [dataR, dataL] = timeSync(dataR, dataL)
i = 0;
    while i < min( length(dataR(:,1)), length(dataL(:,1)) )
        
        i = i + 1;
        if (dataR(i,1) - dataL(i,1)) > 10
            
            j = i + 1;
            while (dataR(i,1) - dataL(j,1)) > 10 && j < min( length(dataR(:,1)), length(dataL(:,1)) )
                j = j + 1;
            end
            dataL(i:end+i-j,:) = dataL(j:end,:);
            dataL(end+i-j+1:end,:) = [];

        end
        
    end
    dataL(length(dataR(:,1))+1:end,:) = [];