function [dataR, dataL, dataC] = timeSync1(dataR, dataL, dataC)
i = 0;
m = min( length(dataR(:,1)), length(dataL(:,1)) )-1;
while i < m
        
    i = i + 1;
    if (dataR(i,1) - dataL(i,1)) > 15

        j = i + 1;
        while (dataR(i,1) - dataL(j,1)) > 15 && j < m
            j = j + 1;
        end
        dataL(i:end+i-j,:) = dataL(j:end,:);
        dataC(i:end+i-j,:) = dataC(j:end,:);
        dataL(end+i-j+1:end,:) = [];
        dataC(end+i-j+1:end,:) = [];
        m = min( length(dataR(:,1)), length(dataL(:,1)) )-1;
        
    end
        
end
dataL(length(dataR(:,1))+1:end,:) = [];
dataC(length(dataR(:,1))+1:end,:) = [];
dataR(length(dataL(:,1))+1:end,:) = [];