--read_register(1,2) --> 1: 확인하고 싶은 모듈 , 2: 확인하고싶은 레지스터 값

-- dis file path input
cpu0_dis_path="C:\\Users\\eyeni\\Desktop\\profiler\\cpu0_vf.dis"
cpu1_dis_path="C:\\Users\\eyeni\\Desktop\\profiler\\cpu1_vf.dis"

--profiler path
pro_path="C:\\Users\\eyeni\\Desktop\\profiler\\"

cnt=0 -- 비교하는 레지스터 값이 같을 때 10번까지 체크해서 같으면 죽은것을 판별하기 위해
cpu0_flag=0
cpu1_flag=0
cpu0_reset_flag=0
cpu1_reset_flag=0
old_cpu0=read_register('isp',0x40048)-- cpu0 PC값
old_cpu1=read_register('ip0',0x40048)-- cpu1 PC값
old_sensor=read_register('wbisp',0xf110091c) -- isp sync sensor 값
old_h264=read_register('wbcom',0xf8f00110) -- h264 사이즈값
old_jpeg=read_register('wbisp',0xf1100a84) -- old_jpeg 사이즈값

while(1)
do
	st=state() -- ui 에서 무한 루프를 강제로 멈출수 있게 하는 함수
	if(st==1)then 
		
		local start = os.time()
		repeat until os.time() > start + 0.1 -- 0.1초 딜레이 
		new_cpu0=read_register("isp",0x40048) 
		new_cpu1=read_register('ip0',0x40048)

		
		if(old_cpu0==new_cpu0)then  -- cpu0동작 이상있을떄
			
			while(old_cpu0==new_cpu0)
			do
				local start = os.time()
				repeat until os.time() > start + 0.1 -- 0.1초 딜레이 
				new_cpu0=read_register("isp",0x40048) 
				cnt=cnt+1
				if(cnt==10) then -- 10번 체크했을 동안 레지스터값이 같으면 죽은걸로 판별
					if(cpu0_flag==0) then -- 죽었을때 파일 한번만 생성할 수 있게 flag 설정
						print('cpu0 error')
						local val_old_cpu0=string.format("old_cpu0=0x%x",old_cpu0) -- 16진수로 출력하기 위해
						local val_new_cpu0=string.format("new_cpu0=0x%x",new_cpu0)
						print(val_old_cpu0)
						print(val_new_cpu0)
						cpu0_flag=1
						cpu0_path=debug_file('isp') --cpu0 tbp값과 dcr값 파일로 생성	


						--local start = os.time()
						--repeat until os.time() > start +1 -- 0.1초 딜레이 

						cpu0_reset_flag=profiler_gen("isp",cpu0_path,cpu0_dis_path,pro_path)
						
						--local start = os.time()
						--repeat until os.time() > start + 1-- 0.1초 딜레이 
						cnt=0 
						break
					else
						break
					end
				end
			end
			cnt=0
		end


		if(old_cpu1==new_cpu1)then  -- cpu1동작 이상있을떄
			while(old_cpu1==new_cpu1) 
			do
				local start = os.time()
				repeat until os.time() > start + 0.1 -- 0.1초 딜레이 
				new_cpu1=read_register('ip0',0x40048)
				cnt=cnt+1
				if(cnt==10) then
					if(cpu1_flag==0) then 
						print('cpu1 error')
						local val_old_cpu1=string.format("old_cpu1=0x%x",old_cpu1) -- 16진수로 출력하기 위해
						local val_new_cpu1=string.format("new_cpu1=0x%x",new_cpu1)
						print(val_old_cpu1)
						print(val_new_cpu1)
						cpu1_flag=1
						cpu1_path=debug_file('ip0')	

						--local start = os.time()
						--repeat until os.time() > start + 0.1-- 0.1초 딜레이 

						cpu1_reset_flag=profiler_gen("ip0",cpu1_path,cpu1_dis_path,pro_path)
						
						--local start = os.time()
						--repeat until os.time() > start + 0.1 -- 0.1초 딜레이 
						cnt=0 
						break
					else
						break
					end
				end
			end

			cnt=0	
		end		
		
		if(cpu0_reset_flag==1 or cpu1_reset_flag==1)then
			
			cpu0_reset_flag=0
			cpu1_reset_flag=0
			cpu1_flag=0
			cpu1_flag=0
			local start = os.time()
			repeat until os.time() > start + 2 -- 1초 딜레이 
			cmd('rst')


		end


		if(old_cpu0~=new_cpu0 and old_cpu1~=new_cpu1) then
			cnt=0
			if(cpu0_flag==1) then
			cpu0_flag=0	
			end
			
			if(cpu1_flag==1) then
			cpu1_flag=0	
			end
		

		new_sensor=read_register('wbisp',0xf110091c) -- 새로운 isp 레지스터 확인
		new_h264=read_register('wbcom',0xf8f00110) --새로운 h264 동작 레지스터 확인
		new_jpeg=read_register('wbisp',0xf1100a84) --새로운 jpeg 동작 레지스터 확인

		local val_old_sensor=string.format("old_sensor=0x%x",old_sensor) -- 오류가 났을때 16진수로 출력하기 위해
		local val_new_sensor=string.format("new_sensor=0x%x",new_sensor)
		local val_old_h264=string.format("old_h264=0x%x",old_h264)
		local val_new_h264=string.format("new_h264=0x%x",new_h264)
		local val_old_jpeg=string.format("old_jpeg=0x%x",old_jpeg)
		local val_new_jpeg=string.format("new_jpeg=0x%x",new_jpeg)
		


		if(old_cpu0~=new_cpu0 and old_sensor~=new_sensor and old_h264~=new_h264 and old_jpeg~=new_jpeg ) then
			print("debugging..")                                                                                                               --이상이 없을때 스크립트가 동작중인것을 표현하기 위해
		end


	
		
			if(old_sensor==new_sensor)then  -- isp동작 이상 있을때
				
				print("isp sync error")			
				print(val_old_sensor)
				print(val_new_sensor)
			
				
				
			end	

			if(old_h264==new_h264)then  -- h264인코딩이 문제 있을떄
				
				
				print("h264 error")		
				print(val_old_h264)
				print( val_new_h264)
				
			
			end

			if(old_jpeg==new_jpeg)then  -- jpeg동작 문제 있을 때

				
				print("jpeg error")
				print(val_old_jpeg)
				print(val_new_jpeg)
				
			
			
			end
			
		
	 			
			
		end		

		old_cpu0=new_cpu0     -- while문 돌면서 지속적으로 비교하기 위해서 old변수에 new  변수 대입
		old_cpu1=new_cpu1
		old_sensor=new_sensor
		old_h264=new_h264
		old_jpeg=new_jpeg


		
	else  -- Lua ui에서 stop을 누를때 스크립트를 멈추게 하기 위해서 


	break

	end



end