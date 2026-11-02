"""Read-only AST inventory; fail when active source imports retired V1 business."""
from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path


RETIRED = {'AI_MVP', 'v0_mvp.pipeline', 'integration.semantic_gateway', 'integration.v0_pick_adapter',
           *('operator_console.' + name for name in (
               'runtime','real_sequence','task_plan','task_plan_model','destination_resolver',
               'semantic','resolution','contracts','modes','profiles','flow','blocked','tools','task_schema_adapter'))}


def audit(root):
    root = Path(root).resolve()
    modules = {}
    for path in sorted((root/'src').rglob('*.py')):
        name = '.'.join(path.relative_to(root/'src').with_suffix('').parts)
        if name.endswith('.__init__'): name = name[:-9]
        modules[name] = path
    graph, imports, violations = {}, [], []
    for name,path in modules.items():
        source = path.read_text(encoding='utf-8-sig')
        tree = ast.parse(source,filename=str(path))
        base = name if path.name == '__init__.py' else name.rpartition('.')[0]
        edges = set()
        # Importing a module always executes its parent package initializers.
        edges.update('.'.join(name.split('.')[:i]) for i in range(1,len(name.split('.')))
                     if '.'.join(name.split('.')[:i]) in modules)
        for node in ast.walk(tree):
            if isinstance(node,ast.Import):
                targets = [a.name for a in node.names]
            elif isinstance(node,ast.ImportFrom):
                prefix = '.'.join(base.split('.')[:len(base.split('.'))-node.level+1]) if node.level else ''
                target = '.'.join(part for part in (prefix,node.module) if part)
                targets = [target] + [target+'.'+a.name for a in node.names]
            else:
                continue
            record = {'file':path.relative_to(root).as_posix(),'line':node.lineno,
                      'statement':ast.get_source_segment(source,node),'targets':targets}
            imports.append(record)
            for target in targets:
                target = target.removeprefix('src.')
                if target in RETIRED:
                    violations.append(record)
                if target in modules:
                    edges.add(target)
        graph[name] = sorted(edges)
    seen, todo = set(), ['operator_console.cli','operator_console.server','operator_console.runtime_v2']
    while todo:
        name = todo.pop()
        if name in seen: continue
        seen.add(name);todo.extend(set(graph.get(name,[]))-seen)
    tests = sorted((root/'tests').rglob('*.py'))
    for path in tests: ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path))
    return {'source_module_count':len(modules),'source_import_count':len(imports),
            'test_python_file_count':len(tests),
            'config_files':sorted(p.relative_to(root).as_posix() for p in (root/'config').rglob('*') if p.is_file()),
            'production_reachable_count':len(seen),'production_reachable':sorted(seen),
            'forbidden_v1_import_count':len(violations),'forbidden_v1_imports':violations,
            'dependency_graph':graph,'source_imports':imports,
            'limits':'AST import inventory, including function-local imports; not a hardware/call/data-flow proof.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    result = audit(args.root)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k.endswith('_count')},ensure_ascii=False))
    return 1 if result['forbidden_v1_import_count'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
