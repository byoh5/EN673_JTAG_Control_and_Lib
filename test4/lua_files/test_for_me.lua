flag=0
cnt=0

while(1)
do
local start = os.time()
repeat until os.time() > start + 0.1 -- 0.5√  µÙ∑π¿Ã 


while(cnt<11) do
local start = os.time()
repeat until os.time() > start + 0.1 -- 0.5√  µÙ∑π¿Ã 
cnt=cnt+1
print(cnt)
local val_flag=string.format("flag=%d", flag)
print(val_flag)
if(cnt==10)then

flag=1
local val_flag=string.format("flag!!!!!!!!!!!!=%d", flag)
print(val_flag)
cnt=0
end

end





end