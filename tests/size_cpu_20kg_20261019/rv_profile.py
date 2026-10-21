"""Instruction-path comparison of the actual RV32EC planner, not a wall-clock MCU timing claim."""
from pathlib import Path
import importlib.util,subprocess,re,struct,json,collections,ctypes
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'output/rv'
spec=importlib.util.spec_from_file_location('b',ROOT/'tests/verify_alignment.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
REG={'zero':0,'ra':1,'sp':2,'gp':3,'tp':4,'t0':5,'t1':6,'t2':7,'s0':8,'s1':9,**{'a'+str(i):10+i for i in range(8)},**{'s'+str(i):16+i for i in range(2,12)},**{'t'+str(i):25+i for i in range(3,7)}}
def signed(x):return x-0x100000000 if x&0x80000000 else x
def number(s):return int(s.strip(),0)

class RV:
    def __init__(self,elf):
        raw=elf.read_bytes();assert raw[:5]==b'\x7fELF\x01';self.mem={};self.reg=[0]*32
        off=struct.unpack_from('<I',raw,32)[0];ent,count=struct.unpack_from('<HH',raw,46)
        for i in range(count):
            _,ty,flags,addr,fo,sz,*_=struct.unpack_from('<10I',raw,off+i*ent)
            if flags&2:
                content=raw[fo:fo+sz] if ty!=8 else bytes(sz)
                self.mem.update({addr+k:v for k,v in enumerate(content)})
        asm=subprocess.check_output([str(b.WCH/'riscv-wch-elf-objdump.exe'),'-d','-M','no-aliases,xw',str(elf)],text=True)
        elf.with_suffix('.dis').write_text(asm);self.code={}
        for line in asm.splitlines():
            m=re.match(r'\s*([0-9a-f]+):\s+([0-9a-f]+)\s+([a-z0-9_.]+)\s*(.*)',line)
            if m:self.code[int(m[1],16)]=(len(m[2])//2,m[3],m[4].split('#')[0].strip())
        nm=subprocess.check_output([str(b.WCH/'riscv-wch-elf-nm.exe'),str(elf)],text=True)
        self.symbols={q[2]:int(q[0],16) for line in nm.splitlines() if len(q:=line.split())==3}
    def read(self,a,n):return sum(self.mem.get(a+i,0)<<(8*i) for i in range(n))
    def write(self,a,x,n):
        for i in range(n):self.mem[a+i]=(x>>(8*i))&255
    def call(self,name,*args):
        self.reg=[0]*32;self.reg[2]=0x20010000;self.reg[1]=0xfffffffc;self.reg[3]=self.symbols.get('__global_pointer$',0)
        self.reg[10:10+len(args)]=[int(v)&0xffffffff for v in args]
        pc=self.symbols[name];count=0;minsp=self.reg[2];hist=collections.Counter();calls=collections.Counter()
        while pc!=0xfffffffc:
            if pc not in self.code:raise ValueError(('bad pc',hex(pc),name,count))
            size,op,operands=self.code[pc];nextpc=pc+size;z=[x.strip() for x in operands.split(',')];r=self.reg
            def rv(x):return r[REG[x]]
            def put(x,v):r[REG[x]]=v&0xffffffff
            def target(x):return int(x.split()[0],16)
            def address(x):
                m=re.fullmatch(r'(-?(?:0x[0-9a-f]+|\d+))\((\w+)\)',x);assert m,x
                return (rv(m[2])+number(m[1]))&0xffffffff
            bare=op[2:] if op.startswith('c.') else op
            if bare in ('addi','addi16sp','addi4spn'):
                if len(z)==2:put(z[0],rv(z[0])+number(z[1]))
                else:put(z[0],rv(z[1])+number(z[2]))
            elif bare=='li':put(z[0],number(z[1]))
            elif bare=='lui':put(z[0],number(z[1])<<12)
            elif bare=='auipc':put(z[0],pc+(number(z[1])<<12))
            elif bare=='mv':put(z[0],rv(z[1]))
            elif bare in ('add','sub','and','or','xor','sll','srl','sra','slt','sltu','mul','mulh','mulhu','mulhsu'):
                a=rv(z[0]) if len(z)==2 else rv(z[1]);bb=rv(z[-1]);sa,sb=signed(a),signed(bb)
                value={'add':lambda:a+bb,'sub':lambda:a-bb,'and':lambda:a&bb,'or':lambda:a|bb,'xor':lambda:a^bb,'sll':lambda:a<<(bb&31),'srl':lambda:a>>(bb&31),'sra':lambda:sa>>(bb&31),'slt':lambda:int(sa<sb),'sltu':lambda:int(a<bb),'mul':lambda:a*bb,'mulh':lambda:(sa*sb)>>32,'mulhu':lambda:(a*bb)>>32,'mulhsu':lambda:(sa*bb)>>32}[bare]()
                put(z[0],value)
            elif bare in ('andi','ori','xori','slli','srli','srai','slti','sltiu'):
                a=rv(z[0]) if len(z)==2 else rv(z[1]);imm=number(z[-1]);sa=signed(a)
                value={'andi':lambda:a&imm,'ori':lambda:a|imm,'xori':lambda:a^imm,'slli':lambda:a<<(imm&31),'srli':lambda:a>>(imm&31),'srai':lambda:sa>>(imm&31),'slti':lambda:int(sa<imm),'sltiu':lambda:int(a<(imm&0xffffffff))}[bare]()
                put(z[0],value)
            elif bare in ('lw','lwsp','lh','lhu','lhusp','lb','lbu'):
                n=4 if bare.startswith('lw') else 2 if bare.startswith('lh') else 1;v=self.read(address(z[1]),n)
                if bare in ('lh','lb') and v&(1<<(8*n-1)):v-=1<<(8*n)
                put(z[0],v)
            elif bare in ('sw','swsp','sh','sb'):self.write(address(z[1]),rv(z[0]),4 if bare.startswith('sw') else 2 if bare=='sh' else 1)
            elif bare in ('beq','bne','blt','bge','bltu','bgeu','beqz','bnez'):
                a=rv(z[0]);bb=0 if bare in ('beqz','bnez') else rv(z[1]);sa,sb=signed(a),signed(bb)
                taken={'beq':a==bb,'bne':a!=bb,'blt':sa<sb,'bge':sa>=sb,'bltu':a<bb,'bgeu':a>=bb,'beqz':a==0,'bnez':a!=0}[bare]
                if taken:nextpc=target(z[-1])
            elif bare in ('j','jal'):
                rd='ra' if bare=='jal' and len(z)==1 else 'zero' if bare=='j' else z[0]
                nextpc=target(z[-1]);put(rd,pc+size);calls[nextpc]+=1
            elif bare in ('jr','jalr'):
                if bare=='jr':rd,src,imm='zero',z[0],0
                elif op=='c.jalr':rd,src,imm='ra',z[0],0
                elif len(z)==3:rd,src,imm=z[0],z[1],number(z[2])
                else:
                    rd=z[0];m=re.fullmatch(r'(-?\d+)\((\w+)\)',z[1]);assert m,z;src=m[2];imm=number(m[1])
                nextpc=(rv(src)+imm)&0xfffffffe;put(rd,pc+size);calls[nextpc]+=1
            elif bare=='nop':pass
            else:raise NotImplementedError((hex(pc),op,operands))
            r[0]=0;pc=nextpc;count+=1;minsp=min(minsp,r[2]);hist[op]+=1
            if count>10000000:raise RuntimeError('instruction budget exceeded')
        return {'instructions':count,'stack_bytes':0x20010000-minsp,'mul_instructions':sum(v for k,v in hist.items() if 'mul' in k),'lib64_div_calls':sum(calls.get(self.symbols.get(n,-1),0) for n in ('__divdi3','__udivdi3')),'return':signed(self.reg[10])}

def build(project,label):
    OUT.mkdir(parents=True,exist_ok=True);obj=OUT/(label+'.o');elf=OUT/(label+'.elf')
    args=[str(b.CC),*b.FLAGS,'-fstack-usage','-I'+str(project/'Common'),'-c',str(project/'Common/C_Traj_Planner.c'),'-o',str(obj)]
    subprocess.run(args,check=True,capture_output=True)
    p=subprocess.run([str(b.CC),*b.FLAGS,'-nostdlib','-Wl,-Ttext=0x1000,-Tdata=0x20000000,--entry=C_Traj_Init,--no-relax','-o',str(elf),str(obj),'-lc','-lgcc'],capture_output=True,text=True)
    assert p.returncode==0,p.stderr
    return elf

def scenario(elf,mode,at=700,T=1000,goal=18500,native=None):
    rv=RV(elf);rv.call('C_Traj_Init',13500);rv.call('C_Traj_SetRange',0,27000)
    rv.call('C_Traj_SetLimits',17100,420000,350000,8400000);rv.call('C_Traj_SetMoveMinMs',150)
    rv.call('C_Traj_Plan',18000,1000)
    if native:
        native['reset'](17100,420000,350000,8400000,150);native['command'](18000,1000,0)
    ref=0x20008000
    def step():
        result=rv.call('C_Traj_Step',ref,0)
        output=[signed(rv.read(ref+4*i,4)) for i in range(3)]+[rv.read(ref+12,1)]
        if native:
            expected=(ctypes.c_int32*4)();native['step'](0,expected);assert list(expected)==output,(list(expected),output)
        return result,output
    for _ in range(at):step()
    if native:native['command'](goal,T,1)
    if mode=='plan':result=rv.call('C_Traj_PlanTimed',goal,T)
    else:
        rv.call('C_Traj_PlanTimed',goal,T)
        for _ in range(10):step()
        result,result['ref']=step()
    if mode=='plan':
        _,result['next_ref']=step()
    return result

def main():
    programs={n:build(p,n) for n,p in [('before',HERE/'before/CH32V00X_APP_ADC_20kg'),('after',HERE/'candidate')]}
    native={}
    for name,prefix in [('before','B'),('after','A')]:
        dll=OUT/(prefix+'.dll')
        subprocess.run([r'D:\SoftWare\mingw64\bin\gcc.exe','-shared','-static-libgcc','-Wl,--export-all-symbols',str(HERE/'output'/(prefix+'.o')),'-o',str(dll)],check=True,capture_output=True)
        lib=ctypes.CDLL(str(dll));native[name]={n:getattr(lib,prefix+'_probe_'+n) for n in ('reset','command','step')}
        native[name]['reset'].argtypes=[ctypes.c_int32]*5;native[name]['reset'].restype=None
        native[name]['command'].argtypes=[ctypes.c_int32]*3;native[name]['command'].restype=None
        native[name]['step'].argtypes=[ctypes.c_int32,ctypes.POINTER(ctypes.c_int32)];native[name]['step'].restype=None
    cases=[('join1000','plan',700,1000,18500),('join3000','plan',700,3000,21000),('short_fallback','plan',150,20,23000),('long65535','plan',300,65535,20000),('poly_step','step',700,1000,18500)]
    result={}
    for name,mode,at,T,goal in cases:
        result[name]={label:scenario(elf,mode,at,T,goal,native[label]) for label,elf in programs.items()};print(name,result[name],flush=True)
        if mode=='step':assert result[name]['before']['ref']==result[name]['after']['ref']
    (OUT/'results.json').write_text(json.dumps(result,indent=2))
if __name__=='__main__':main()
