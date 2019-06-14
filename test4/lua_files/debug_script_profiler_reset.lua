--read_register(1,2) --> 1: Ȯ���ϰ� ���� ��� , 2: Ȯ���ϰ���� �������� ��

-- dis file path input
cpu0_dis_path="C:\\Users\\eyeni\\Desktop\\profiler\\cpu0_vf.dis"
cpu1_dis_path="C:\\Users\\eyeni\\Desktop\\profiler\\cpu1_vf.dis"

--profiler path
pro_path="C:\\Users\\eyeni\\Desktop\\profiler\\"

cnt=0 -- ���ϴ� �������� ���� ���� �� 10������ üũ�ؼ� ������ �������� �Ǻ��ϱ� ����
cpu0_flag=0
cpu1_flag=0
cpu0_reset_flag=0
cpu1_reset_flag=0
old_cpu0=read_register('isp',0x40048)-- cpu0 PC��
old_cpu1=read_register('ip0',0x40048)-- cpu1 PC��
old_sensor=read_register('wbisp',0xf110091c) -- isp sync sensor ��
old_h264=read_register('wbcom',0xf8f00110) -- h264 �����
old_jpeg=read_register('wbisp',0xf1100a84) -- old_jpeg �����

while(1)
do
	st=state() -- ui ���� ���� ������ ������ ����� �ְ� �ϴ� �Լ�
	if(st==1)then 
		
		local start = os.time()
		repeat until os.time() > start + 0.1 -- 0.1�� ������ 
		new_cpu0=read_register("isp",0x40048) 
		new_cpu1=read_register('ip0',0x40048)

		
		if(old_cpu0==new_cpu0)then  -- cpu0���� �̻�������
			
			while(old_cpu0==new_cpu0)
			do
				local start = os.time()
				repeat until os.time() > start + 0.1 -- 0.1�� ������ 
				new_cpu0=read_register("isp",0x40048) 
				cnt=cnt+1
				if(cnt==10) then -- 10�� üũ���� ���� �������Ͱ��� ������ �����ɷ� �Ǻ�
					if(cpu0_flag==0) then -- �׾����� ���� �ѹ��� ������ �� �ְ� flag ����
						print('cpu0 error')
						local val_old_cpu0=string.format("old_cpu0=0x%x",old_cpu0) -- 16������ ����ϱ� ����
						local val_new_cpu0=string.format("new_cpu0=0x%x",new_cpu0)
						print(val_old_cpu0)
						print(val_new_cpu0)
						cpu0_flag=1
						cpu0_path=debug_file('isp') --cpu0 tbp���� dcr�� ���Ϸ� ����	


						--local start = os.time()
						--repeat until os.time() > start +1 -- 0.1�� ������ 

						cpu0_reset_flag=profiler_gen("isp",cpu0_path,cpu0_dis_path,pro_path)
						
						--local start = os.time()
						--repeat until os.time() > start + 1-- 0.1�� ������ 
						cnt=0 
						break
					else
						break
					end
				end
			end
			cnt=0
		end


		if(old_cpu1==new_cpu1)then  -- cpu1���� �̻�������
			while(old_cpu1==new_cpu1) 
			do
				local start = os.time()
				repeat until os.time() > start + 0.1 -- 0.1�� ������ 
				new_cpu1=read_register('ip0',0x40048)
				cnt=cnt+1
				if(cnt==10) then
					if(cpu1_flag==0) then 
						print('cpu1 error')
						local val_old_cpu1=string.format("old_cpu1=0x%x",old_cpu1) -- 16������ ����ϱ� ����
						local val_new_cpu1=string.format("new_cpu1=0x%x",new_cpu1)
						print(val_old_cpu1)
						print(val_new_cpu1)
						cpu1_flag=1
						cpu1_path=debug_file('ip0')	

						--local start = os.time()
						--repeat until os.time() > start + 0.1-- 0.1�� ������ 

						cpu1_reset_flag=profiler_gen("ip0",cpu1_path,cpu1_dis_path,pro_path)
						
						--local start = os.time()
						--repeat until os.time() > start + 0.1 -- 0.1�� ������ 
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
			repeat until os.time() > start + 2 -- 1�� ������ 
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
		

		new_sensor=read_register('wbisp',0xf110091c) -- ���ο� isp �������� Ȯ��
		new_h264=read_register('wbcom',0xf8f00110) --���ο� h264 ���� �������� Ȯ��
		new_jpeg=read_register('wbisp',0xf1100a84) --���ο� jpeg ���� �������� Ȯ��

		local val_old_sensor=string.format("old_sensor=0x%x",old_sensor) -- ������ ������ 16������ ����ϱ� ����
		local val_new_sensor=string.format("new_sensor=0x%x",new_sensor)
		local val_old_h264=string.format("old_h264=0x%x",old_h264)
		local val_new_h264=string.format("new_h264=0x%x",new_h264)
		local val_old_jpeg=string.format("old_jpeg=0x%x",old_jpeg)
		local val_new_jpeg=string.format("new_jpeg=0x%x",new_jpeg)
		


		if(old_cpu0~=new_cpu0 and old_sensor~=new_sensor and old_h264~=new_h264 and old_jpeg~=new_jpeg ) then
			print("debugging..")                                                                                                               --�̻��� ������ ��ũ��Ʈ�� �������ΰ��� ǥ���ϱ� ����
		end


	
		
			if(old_sensor==new_sensor)then  -- isp���� �̻� ������
				
				print("isp sync error")			
				print(val_old_sensor)
				print(val_new_sensor)
			
				
				
			end	

			if(old_h264==new_h264)then  -- h264���ڵ��� ���� ������
				
				
				print("h264 error")		
				print(val_old_h264)
				print( val_new_h264)
				
			
			end

			if(old_jpeg==new_jpeg)then  -- jpeg���� ���� ���� ��

				
				print("jpeg error")
				print(val_old_jpeg)
				print(val_new_jpeg)
				
			
			
			end
			
		
	 			
			
		end		

		old_cpu0=new_cpu0     -- while�� ���鼭 ���������� ���ϱ� ���ؼ� old������ new  ���� ����
		old_cpu1=new_cpu1
		old_sensor=new_sensor
		old_h264=new_h264
		old_jpeg=new_jpeg


		
	else  -- Lua ui���� stop�� ������ ��ũ��Ʈ�� ���߰� �ϱ� ���ؼ� 


	break

	end



end