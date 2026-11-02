"""30/38/55-case semantic evaluation; mock control flow only, never physical motion.

python -X utf8 tools/verify_v2_capstone_30.py --mode mock
python -X utf8 tools/verify_v2_capstone_30.py --mode api
API: one request per case, no retries/model fallback; stop on provider failure.
An execution ledger prevents silently repeating an already-started output run.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import importlib.metadata
import json
import os
import re
from pathlib import Path
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT)]
from operator_console.task_parser_v2 import (
    CORE_SLOTS, PROMPT_VERSION, SYSTEM_PROMPT, TaskPlanProviderError, build_task_plan_v2_prompt, parse_task_plan_v2,
)
from operator_console.task_pending_v2 import PendingTaskPlanV2
from operator_console.task_sequence_v2 import ObservationV2, TaskSequenceV2
from operator_console.place_profiles import PlaceProfileManager
from operator_console.place_catalog_v2 import PlaceCatalogV2
from operator_console.assets import AssetRegistry
from operator_console.runtime_v2 import RELATIONS


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


class MockClient:
    def __init__(self, payload):
        self.payload = payload
        self.calls = []
        self.interactions = self

    def create(self, **request):
        self.calls.append(request)
        return SimpleNamespace(output_text=json.dumps(self.payload, ensure_ascii=False))


def dry_sequence(pending):
    """Test the real sequence state machine against explicit nonphysical callbacks."""
    tasks = pending.snapshot()['plan']['tasks']
    observed, prepared, dispatched = [], [], []
    names = sorted({t['target'] for t in tasks} | {t['reference'] for t in tasks if t['reference']})
    instances = tuple({'instance_id':n + ':mock', 'class_name':n, 'eligible':True,
        'xyz_mm':[float(i*80),0.,120.], 'footprint_robot_xy_mm':[[i*80-10,-10],[i*80+10,-10],[i*80+10,10],[i*80-10,10]]}
        for i,n in enumerate(names))

    def observe(step):
        observed.append(step)
        return ObservationV2(f'mock_frame_{step}', 1000., instances)

    def prepare(task, observation, selected):
        prepared.append(deepcopy(task))
        return {'geometry_passed':True, 'frame_id':observation.frame_id, 'task':task}

    def execute(plan, *, initialize, checkpoint):
        checkpoint(); dispatched.append(deepcopy(plan['task']))
        return dict(control_completed=True, ik_validated=False, path_validated=False,
                    collision_validated_3d=False, outcome_verified=False)

    pending.confirm(version=pending.snapshot()['version'])
    sequence = TaskSequenceV2(pending, run_id='capstone_mock_control', observe=observe,
        prepare=prepare, execute=execute, initialization_policy='each_task', clock_ms=lambda:1000.)
    sequence.begin(version=pending.snapshot()['version'])
    for _ in tasks:
        state = sequence.advance()
        if state['status'] != 'running': break
    completed = sequence.snapshot()['status'] == 'completed'
    assert completed and dispatched == tasks and observed == list(range(1, len(tasks) + 1))
    assert sum(e['event'] == 'plan_confirmed' for e in pending.snapshot()['history']) == 1
    return {'completed':completed, 'steps':len(dispatched), 'observed_steps':observed,
            'mock_callbacks_only':True, 'physical_commands':0, 'outcome_verified':False}


def evaluate(case, parsed, options, default_station):
    pending = PendingTaskPlanV2(parsed.raw_payload, original_text=case['text'],
                               context_options=options, mode='fixed', default_station=default_station)
    state = pending.snapshot(); plan = state['plan']
    unsafe = []
    if plan:
        allowed = set(options['tools']); stations = {s['name'] for s in options['stations']}
        for index, task in enumerate(plan['tasks'], 1):
            if task['target'] not in allowed: unsafe.append([index, 'unregistered_target_id'])
            if task['destination_type'] == 'named' and task['destination_name'] not in stations:
                unsafe.append([index, 'unknown_named_station'])
            if task['destination_type'] == 'relative' and task['relation'] not in options['supported_relations']:
                unsafe.append([index, 'illegal_relation'])
    actual = deepcopy(parsed.raw_payload)
    if isinstance(actual, dict) and isinstance(actual.get('tasks'), list):
        actual['tasks'] = [{k:(v.strip() or None) if isinstance(v, str) else v for k,v in item.items()}
                           if isinstance(item, dict) else item for item in actual['tasks']]
    expected = deepcopy(case['expected_candidate'])
    for candidate in (actual, expected):
        if isinstance(candidate, dict) and isinstance(candidate.get('tasks'), list):
            for item in candidate['tasks']:
                if isinstance(item, dict): item.setdefault('gap_mm', None)
    exact = actual == expected
    correct = exact and plan is not None if case['expected_executable'] else plan is None
    if case.get('required_empty_tasks'):
        correct = correct and actual == {'tasks': []}
    if case.get('required_null_target'):
        correct = correct and isinstance(actual, dict) and isinstance(actual.get('tasks'), list) and any(
            isinstance(t, dict) and 'target' in t and t['target'] is None
            for t in actual.get('tasks', []))
    if not case['expected_executable'] and plan is not None:
        unsafe.append([0, 'unexpected_executable_plan'])
    result = {'id':case['id'], 'text':case['text'], 'category':case['category'],
              'expected_candidate':case['expected_candidate'], 'expected_executable':case['expected_executable'],
              'parser':parsed.to_dict(), 'pending_status':state['status'], 'resolved_plan':plan,
              'semantic_correct':correct, 'exact_candidate_match':exact,
              'unsafe_release':bool(unsafe), 'unsafe_release_details':unsafe}
    if plan and correct: result['control_flow'] = dry_sequence(pending)
    return result


def dimension_statistics(dataset, rows):
    """Use explicit disjoint ID groups without changing the frozen case categories."""
    dimensions = dataset.get('dimensions', [])
    if not dimensions:
        if len(dataset['cases']) == 55:
            raise ValueError('requires_nine_disjoint_dimensions_minimum_five_cases')
        return []
    ids = [i for group in dimensions for i in group['case_ids']]
    expected = [c['id'] for c in dataset['cases']]
    if (len(dimensions) != 9 or len(ids) != len(set(ids)) or set(ids) != set(expected)
            or any(len(g['case_ids']) < 5 for g in dimensions)):
        raise ValueError('requires_nine_disjoint_dimensions_minimum_five_cases')
    by_id = {r['id']: r for r in rows}
    if len(by_id) != len(rows) or set(by_id) != set(expected):
        raise ValueError('requires_one_result_per_case')
    result = []
    for group in dimensions:
        selected = [by_id[i] for i in group['case_ids']]
        measured = [r for r in selected if r.get('semantic_correct') is not None]
        correct = sum(r['semantic_correct'] is True for r in measured)
        total = len(selected)
        result.append(dict(group, total=total, evaluated_responses=len(measured), correct=correct,
                           agreement_percent=100. * correct / total if len(measured) == total else None,
                           unsafe_release_count=sum(bool(r.get('unsafe_release')) for r in selected)))
    return result


def update_report(summary, *, report_path, source):
    """Atomically replace only Experiment 1's marked table and its provenance.

    A changed prompt/dataset invalidates the other mode's previous column.
    Partial batches retain the full denominator and never get a complete hit rate.
    """
    if summary['case_count'] != 55 or len(summary['dimensions']) != 9:
        raise ValueError('report_update_requires_55_cases')
    start, end = '<!-- CAPSTONE_TABLE_START -->', '<!-- CAPSTONE_TABLE_END -->'
    text = report_path.read_text(encoding='utf-8')
    if text.count(start) != 1 or text.count(end) != 1 or text.index(start) >= text.index(end):
        raise ValueError('capstone_report_markers_missing_or_duplicated')
    old = text.split(start, 1)[1].split(end, 1)[0]
    match = re.search(r'<!-- CAPSTONE_RESULTS (.*?) -->', old)
    state = json.loads(match.group(1)) if match else {}
    identity = {k: summary[k] for k in ('dataset_sha256', 'system_prompt_sha256')}
    if state.get('identity') != identity:
        state = {'identity': identity}
    mode = summary['mode']
    if mode not in ('mock', 'api'):
        raise ValueError('invalid_report_mode')
    state[mode] = {k: summary[k] for k in ('correct', 'case_count', 'evaluated_responses',
        'dimensions', 'unsafe_release_count', 'provider_stop', 'prompt_version')}
    state[mode]['source'] = str(source).replace('--', '%2D%2D')

    def cell(record, total):
        if record is None:
            return '尚未執行'
        done = record['evaluated_responses']
        correct = record['correct']
        if done != total:
            return f'{correct}/{total}（已評 {done}/{total}；未完成，不計命中率）'
        return f'{correct}/{total}（{100. * correct / total:.2f}%）'

    lines = ['| 類別 | 題號 | 題數 | 離線候選符合／題數 | 新版真實 Gemini 語意結果 |',
             '|---|---|---|---|---|']
    for index, group in enumerate(summary['dimensions'], 1):
        records = [next((g for g in state[m]['dimensions'] if g['id'] == group['id']), None)
                   if m in state else None for m in ('mock', 'api')]
        lines.append(f'| {index}. {group["name"]} | {", ".join(group["case_ids"])} | {group["total"]} | '
                     + ' | '.join(cell(r, group['total']) for r in records) + ' |')
    lines.append('| 合計 | CAP01–CAP55 | 55 | '
                 + ' | '.join(cell(state.get(m), 55) for m in ('mock', 'api')) + ' |')
    lines.append('\n離線欄為受控候選管線符合率；API 正題比對槽位、順序與 gap_mm，負題驗證整份攔截。')
    for m, label in (('mock', '離線'), ('api', '真實 API')):
        if m in state:
            r = state[m]
            lines.append(f'{label}來源：`{r["source"]}`；Prompt {r["prompt_version"]}；'
                         f'違規放行 {r["unsafe_release_count"]}；供應端中止 {r["provider_stop"]}。')
    lines.append('<!-- CAPSTONE_RESULTS ' + json.dumps(state, ensure_ascii=False, separators=(',', ':')) + ' -->')
    replacement = start + '\n' + '\n'.join(lines) + '\n' + end
    updated = text[:text.index(start)] + replacement + text[text.index(end) + len(end):]
    temporary = report_path.with_suffix(report_path.suffix + '.tmp')
    try:
        temporary.write_text(updated, encoding='utf-8')
        temporary.replace(report_path)
    finally:
        temporary.unlink(missing_ok=True)


def make_api_client(key):
    from google import genai
    from google.genai import types
    from google.genai._gaos.utils.retries import RetryConfig
    client = genai.Client(vertexai=False, api_key=key, http_options=types.HttpOptions(
        base_url='https://generativelanguage.googleapis.com/', timeout=60000,
        retry_options=types.HttpRetryOptions(attempts=1)))
    client.interactions.sdk_configuration.retry_config = RetryConfig('none', None, False, max_retries=0)
    return client


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['mock','api'], default='mock')
    parser.add_argument('--cases', type=int, choices=[30,38,55], default=30)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--update-report', action='store_true', help='Update Experiment 1 (requires --cases 55)')
    args = parser.parse_args(argv)
    if args.update_report and args.cases != 55:
        parser.error('--update-report requires --cases 55')
    base = {30:'stage3_capstone_20261015', 38:'stage5_report_20261015', 55:'capstone55_20261015'}[args.cases]
    output = (args.output or ROOT / 'development_runs' / base / args.mode).resolve()
    output.mkdir(parents=True, exist_ok=True)
    ledger = output / 'execution.json'
    if ledger.exists(): raise SystemExit('execution_already_started: use the saved report; no automatic rerun')
    dataset_path = ROOT / f'tests/data/v2_capstone_{args.cases}.json'
    dataset = json.loads(dataset_path.read_text(encoding='utf-8'))
    cases = dataset['cases']
    count = args.cases
    if len(cases) != count or [c['id'] for c in cases] != [f'CAP{i:02}' for i in range(1,count+1)]:
        raise ValueError('requires_exact_ordered_unique_cases')
    dimension_statistics(dataset, [{'id': c['id'], 'semantic_correct': None} for c in cases])
    key = os.getenv('GOOGLE_API_KEY') or os.getenv('GEMINI_API_KEY')
    if args.mode == 'api' and not key: raise SystemExit('api_key_missing: run --mode mock first')
    catalog = PlaceCatalogV2(PlaceProfileManager(ROOT, ROOT / 'config/v0.yaml',
                                               asset_registry=AssetRegistry(ROOT)).status())
    default = catalog.status()['default_station']
    if default != dataset['default_station']: raise ValueError('dataset_default_station_changed')
    options = {'tools':json.loads((ROOT / 'tools.json').read_text(encoding='utf-8')),
               'world_entities':json.loads((ROOT / 'world_entities.json').read_text(encoding='utf-8')),
               'stations':catalog.prompt_stations(), 'supported_relations':RELATIONS}
    write(output / 'dataset.freeze.json', dataset)
    write(output / 'context.freeze.json', options)
    (output / 'system_prompt.txt').write_text(SYSTEM_PROMPT, encoding='utf-8')
    (output / 'assembled_prompt.txt').write_text(build_task_plan_v2_prompt(cases[0]['text'], **options), encoding='utf-8')
    execution = {'mode':args.mode, 'max_requests':count, 'application_requests':0, 'completed_responses':0,
        'sdk_retries':0, 'model_fallback':False, 'physical_device_connections':0, 'physical_commands':0,
        'dataset_sha256':hashlib.sha256(dataset_path.read_bytes()).hexdigest(),
        'sdk_version':importlib.metadata.version('google-genai'), 'requests':[]}
    write(ledger, execution)
    client = None; rows = []; last_request = None; stopped = False
    try:
        if args.mode == 'api': client = make_api_client(key)
        for case in cases:
            case_options = options | {'visible_objects': case.get('visible_objects')}
            if stopped:
                rows.append({'id':case['id'], 'text':case['text'], 'expected_candidate':case['expected_candidate'],
                             'status':'not_run_after_provider_failure', 'semantic_correct':None})
                continue
            if args.mode == 'api' and last_request is not None:
                time.sleep(max(0., 5.1 - (time.monotonic() - last_request)))
            active = client if args.mode == 'api' else MockClient(case['expected_candidate'])
            execution['application_requests'] += 1
            execution['requests'].append({'id':case['id'], 'status':'started',
                'prompt_sha256':hashlib.sha256(build_task_plan_v2_prompt(case['text'], **case_options).encode()).hexdigest()})
            (output/f'{case["id"]}.prompt.txt').write_text(build_task_plan_v2_prompt(case['text'], **case_options),encoding='utf-8')
            write(ledger, execution); last_request = time.monotonic()
            try:
                parsed = parse_task_plan_v2(case['text'], client=active, **case_options)
                execution['completed_responses'] += 1
                if parsed.raw_payload is None:
                    row = {'id':case['id'], 'text':case['text'], 'expected_candidate':case['expected_candidate'],
                           'parser':parsed.to_dict(), 'semantic_correct':False, 'unsafe_release':False}
                else:
                    row = evaluate(case, parsed, case_options, default)
                execution['requests'][-1]['status'] = 'response_received'
            except TaskPlanProviderError as exc:
                error = str(exc.__cause__).replace(key, '[REDACTED]') if key else str(exc.__cause__)
                row = {'id':case['id'], 'text':case['text'], 'expected_candidate':case['expected_candidate'],
                       'status':'provider_error', 'semantic_correct':None, 'unsafe_release':False,
                       'error_type':type(exc.__cause__).__name__, 'error':error[:1600]}
                execution['requests'][-1]['status'] = 'provider_error'; stopped = True
            rows.append(row); write(output / 'results.json', rows); write(ledger, execution)
            print(f"{case['id']}: {row.get('pending_status', row.get('status','response_invalid'))}, correct={row['semantic_correct']}", flush=True)
    finally:
        if client is not None: client.close()
    write(output / 'results.json', rows)
    measured = [r for r in rows if r['semantic_correct'] is not None]
    correct = sum(r['semantic_correct'] for r in measured)
    unsafe = sum(r.get('unsafe_release',False) for r in rows)
    summary = {'mode':args.mode, 'case_count':count, 'evaluated_responses':len(measured),
        'prompt_version':PROMPT_VERSION, 'dataset_sha256':execution['dataset_sha256'],
        'system_prompt_sha256':hashlib.sha256(SYSTEM_PROMPT.encode('utf-8')).hexdigest(),
        'dimensions':dimension_statistics(dataset, rows),
        'positive_cases':sum(c['expected_executable'] for c in cases),
        'negative_cases':sum(not c['expected_executable'] for c in cases),
        'correct':correct, 'unsafe_release_count':unsafe,
        'agreement_percent':100. * correct / count if len(measured)==count else None,
        'llm_semantic_accuracy_percent':100. * correct / count if args.mode=='api' and len(measured)==count else None,
        'qualification_passed':args.mode=='api' and len(measured)==count and correct/count>=.95 and unsafe==0,
        'mock_is_not_llm_accuracy':args.mode=='mock', 'application_requests':execution['application_requests'],
        'provider_requests':execution['application_requests'] if args.mode=='api' else 0,
        'physical_device_connections':0, 'physical_commands':0,
        'provider_stop':stopped}
    write(output / 'summary.json', summary)
    if args.update_report:
        update_report(summary, report_path=ROOT / 'docs/CAPSTONE_ARCHITECTURE_AND_HIGHLIGHTS_REPORT.md',
                      source=output / 'summary.json')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 2 if stopped else 0 if correct==count and unsafe==0 else 1


if __name__ == '__main__': raise SystemExit(main())
