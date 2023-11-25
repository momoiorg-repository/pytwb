from abc import abstractmethod

class Dependency:
    @abstractmethod
    def append(self, log) -> None:
        pass

    @abstractmethod
    def emit_docker_type(self, log) -> list:
        pass

class AptDependency(Dependency):
    name = 'apt'
    apt_init = [
        'vim', 'xterm', 'less', 'git', 'python3-pip', 
        'ros-humble-navigation2', 'ros-humble-py-trees',
        'ros-humble-py-trees-ros'
    ]

    def __init__(self) -> None:
        self.modules = []

    def append(self, log):
        if log.args[0] != 'install': return
        for a in log.args[1:]:
            if a.startswith('-'): continue
            self.modules.append(a)
    
    def emit_docker_type(self):
        apt_list = 'RUN apt-get update && apt-get install -y --no-install-recommends '
        apt_list += ' '.join(self.apt_init + self.modules)
        return apt_list

class Pip3Dependency(Dependency):
    name = 'pip3'

    def __init__(self) -> None:
        self.modules = []

    def append(self, log):
        if log.args[0] != 'install': return
        for a in log.args[1:]:
            if a.startswith('-'): continue
            self.modules.append(a)
    
    def emit_docker_type(self):
        if len(self.modules) < 1: return None
        pip3_list = 'RUN pip3 install '
        pip3_list += ' '.join(self.modules)
        return pip3_list

class EnvDependency(Dependency):
    name = 'env'

    def __init__(self) -> None:
        self.param_table = {}

    def append(self, log):
        self.param_table[log.args[0]] = log.args[1]
    
    def emit_docker_type(self):
        if len(self.param_table) < 1: return None
        param_list = ''    
        for key, value in self.param_table.items():
            param_list += f'ENV {key}={value}\n'
        return param_list

class ParamDependency(Dependency):
    name = 'param'

    def __init__(self) -> None:
        self.param_table = {}

    def append(self, log):
        self.param_table[log.args[0]] = log.args[1]
    
    def emit_docker_type(self):
        pass

def emit_dep_list(config, dep_class):
    dep_table = {}
    dep_list = []
    for dep in dep_class:
        analyzer = dep()
        dep_table[dep.name] = analyzer
        dep_list.append(analyzer)
    for l in config.com_log:
        analyzer = dep_table.get(l.command)
        if not analyzer: continue
        analyzer.append(l)
    rep = ''
    for a in dep_list:
        val = a.emit_docker_type()
        if val: rep += val + '\n'
    return rep

def get_env_table(config):
    env_analyzer = EnvDependency()
    for l in config.com_log:
        if l.command != 'env': continue
        env_analyzer.append(l)
    return env_analyzer.param_table

def get_param_table(config):
    env_analyzer = ParamDependency()
    for l in config.com_log:
        if l.command != 'param': continue
        env_analyzer.append(l)
    return env_analyzer.param_table

def print_com_log(config):
    for l in config.com_log:
        print(f'{l.command} {" ".join(l.args)}')

def emit_dockerfile(config):
    dep_list = emit_dep_list(config, [AptDependency, Pip3Dependency, EnvDependency])
    ws_dir = config._current.ws

    return \
f'''
FROM ros:humble
SHELL ["/bin/bash", "-c"]

{dep_list}

WORKDIR /usr/local/lib
RUN git clone https://github.com/momoiorg-repository/pytwb.git
WORKDIR /usr/local/lib/pytwb
RUN source /opt/ros/humble/setup.bash && pip3 install -e .

WORKDIR /root
COPY ./src {ws_dir}
COPY _.pytwb .pytwb
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc
'''
