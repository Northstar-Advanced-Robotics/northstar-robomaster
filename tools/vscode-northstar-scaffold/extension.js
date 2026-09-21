/*
 * NorthStar Scaffold -- status-bar buttons for generating subsystems and commands.
 *
 * Plain JavaScript on purpose: there is no Node toolchain in the devcontainer,
 * so this extension has no build step. VS Code runs it on its own bundled Node
 * and injects `require('vscode')`. It is installed by symlinking this folder
 * into ~/.vscode-server/extensions/northstar.scaffold-0.0.1 -- see
 * scripts/install-extension.sh.
 *
 * All the real work lives in scripts/scaffold (Python). This file is only a
 * wizard around it, so the behaviour stays identical to the CLI and the
 * "Scaffold - New ..." tasks.
 */

const vscode = require('vscode');
const { execFile } = require('child_process');
const path = require('path');

const SCAFFOLD = 'scripts/scaffold';
const PROJECT_SRC = 'northstar-robomaster-project/src';
const ROBOTS = ['standard', 'hero', 'sentry', 'turret', 'testbed'];

let output;

function activate(context) {
    output = vscode.window.createOutputChannel('NorthStar Scaffold');
    context.subscriptions.push(output);

    const buttons = [
        ['northstar.newSubsystem', '$(circuit-board)', 'New Subsystem', 'subsystem'],
        ['northstar.newCommand', '$(run-all)', 'New Command', 'command'],
    ];

    for (const [id, icon, label, kind] of buttons) {
        const item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100);
        item.text = `${icon} ${label}`;
        item.tooltip = `Scaffold a new Taproot ${kind}`;
        item.command = id;
        item.show();
        context.subscriptions.push(item);

        context.subscriptions.push(
            vscode.commands.registerCommand(id, () =>
                wizard(kind).catch((err) =>
                    vscode.window.showErrorMessage(`Scaffold failed: ${err.message}`))));
    }
}

function deactivate() {}

/** Run the Python scaffolder and parse its --json output. */
function run(cwd, args) {
    return new Promise((resolve, reject) => {
        execFile('python3', [SCAFFOLD, ...args, '--json'], { cwd, maxBuffer: 8 * 1024 * 1024 },
            (err, stdout, stderr) => {
                if (err) {
                    // The CLI reports its own errors on stderr; surface them verbatim.
                    return reject(new Error((stderr || err.message).trim()));
                }
                try {
                    resolve(JSON.parse(stdout));
                } catch (e) {
                    reject(new Error(`could not parse scaffolder output: ${stdout.slice(0, 400)}`));
                }
            });
    });
}

async function listControlDirs(root) {
    try {
        const entries = await vscode.workspace.fs.readDirectory(
            vscode.Uri.joinPath(root, PROJECT_SRC, 'control'));
        return entries.filter(([, type]) => type === vscode.FileType.Directory)
            .map(([name]) => name)
            .sort();
    } catch (e) {
        return [];
    }
}

/** Subsystem classes that are actually defined (not just forward declared). */
async function listSubsystemClasses(root) {
    const files = await vscode.workspace.findFiles(
        `${PROJECT_SRC}/**/*subsystem*.hpp`, '**/taproot/**', 200);
    const found = new Set();
    for (const file of files) {
        try {
            const text = Buffer.from(await vscode.workspace.fs.readFile(file)).toString('utf8');
            // `class X : ...` or `class X {` -- a definition, never `class X;`
            const re = /^[ \t]*class\s+(\w+)\s*(?::|\{)/gm;
            let m;
            while ((m = re.exec(text)) !== null) found.add(m[1]);
        } catch (e) { /* unreadable file, skip */ }
    }
    return [...found].sort();
}

async function wizard(kind) {
    const folder = vscode.workspace.workspaceFolders && vscode.workspace.workspaceFolders[0];
    if (!folder) {
        vscode.window.showErrorMessage('Open the northstar-robomaster folder first.');
        return;
    }
    const root = folder.uri;
    const cwd = root.fsPath;
    const total = kind === 'command' ? 4 : 3;
    let step = 0;

    const name = await vscode.window.showInputBox({
        title: `New ${kind} (${++step}/${total})`,
        prompt: 'Name -- "Flywheel", "flywheel_subsystem" and "FlywheelSubsystem" all work',
        placeHolder: 'Flywheel',
        validateInput: (v) =>
            /^[A-Za-z][A-Za-z0-9 _-]*$/.test(v || '')
                ? null
                : 'Start with a letter; letters, digits, spaces, _ and - only.',
    });
    if (!name) return;

    const NEW_DIR = '$(new-folder) new directory...';
    const dirs = await listControlDirs(root);
    const dirPick = await vscode.window.showQuickPick(
        [{ label: NEW_DIR, alwaysShow: true }, ...dirs.map((label) => ({ label }))],
        {
            title: `New ${kind} (${++step}/${total})`,
            placeHolder: 'Feature directory under src/control/ (Esc to derive it from the name)',
        });
    if (!dirPick) return;

    let directory = dirPick.label;
    if (directory === NEW_DIR) {
        directory = await vscode.window.showInputBox({
            title: 'New feature directory',
            prompt: 'Directory name under src/control/',
            placeHolder: 'winch',
            validateInput: (v) =>
                /^[A-Za-z][A-Za-z0-9_]*$/.test(v || '') ? null : 'Letters, digits and _ only.',
        });
        if (!directory) return;
    }

    let requires;
    if (kind === 'command') {
        const classes = await listSubsystemClasses(root);
        const pick = await vscode.window.showQuickPick(classes, {
            title: `New ${kind} (${++step}/${total})`,
            placeHolder: 'Which subsystem does this command require?',
        });
        if (!pick) return;
        requires = pick;
    }

    // canPickMany is the thing a tasks.json pickString cannot do. `standard` is
    // pre-ticked, so the common case is a single Enter.
    const picks = await vscode.window.showQuickPick(
        ROBOTS.map((label) => ({ label, picked: label === 'standard' })),
        {
            title: `New ${kind} (${++step}/${total})`,
            canPickMany: true,
            placeHolder:
                'Wiring checklist for which robots? (pick none to just create the files)',
        });
    if (!picks) return;
    const robots = picks.map((p) => p.label);

    let args = [kind, name, '--dir', directory, '--robots', robots.join(',') || 'none'];
    if (requires) args = args.concat(['--requires', requires]);

    // Dry run first, so nothing is created until the user has seen the result.
    let preview;
    while (true) {
        preview = await run(cwd, args.concat(['--dry-run']));
        const files = preview.files.map((f) => '  ' + path.relative(cwd, f.path)).join('\n');
        const choice = await vscode.window.showInformationMessage(
            `Create ${preview.names.class_name}?`,
            { modal: true, detail: `namespace ${preview.names.namespace}\n\n${files}` },
            'Create', 'Rename class...');
        if (choice === 'Create') break;
        if (choice !== 'Rename class...') return;

        const renamed = await vscode.window.showInputBox({
            title: 'Class name',
            prompt: 'Useful for acronyms the deriver gets wrong, e.g. DJITwoFlywheelSubsystem',
            value: preview.names.class_name,
            validateInput: (v) =>
                /^[A-Za-z_]\w*$/.test(v || '') ? null : 'Must be a valid C++ identifier.',
        });
        if (!renamed) return;
        args = args.filter((a, i) => a !== '--class' && args[i - 1] !== '--class');
        args = args.concat(['--class', renamed]);
    }

    const result = await run(cwd, args);
    report(result, cwd);

    const header = result.files.find((f) => f.path.endsWith('.hpp'));
    if (header) {
        await vscode.window.showTextDocument(
            await vscode.workspace.openTextDocument(vscode.Uri.file(header.path)));
    }

    const hasWiring = result.wiring.some((w) => w.steps.length);
    const actions = hasWiring ? ['Show wiring steps', 'Copy snippets'] : [];
    const picked = await vscode.window.showInformationMessage(
        `Created ${result.names.class_name}.`, ...actions);

    if (picked === 'Show wiring steps') output.show(true);
    if (picked === 'Copy snippets') {
        await vscode.env.clipboard.writeText(snippetsOnly(result));
        vscode.window.showInformationMessage('Wiring snippets copied to the clipboard.');
    }
}

function snippetsOnly(result) {
    const out = [];
    for (const plan of result.wiring) {
        out.push(`// ---- ${path.basename(plan.control_file)} ----`);
        for (const step of plan.steps) {
            if (step.already_present) continue;
            out.push(`// ${step.where}${step.line ? ` (line ${step.line})` : ''}`);
            out.push(step.snippet);
            out.push('');
        }
    }
    return out.join('\n');
}

function report(result, cwd) {
    output.clear();
    output.appendLine(`Created ${result.names.class_name}  [${result.names.namespace}]`);
    for (const f of result.files) output.appendLine(`  ${path.relative(cwd, f.path)}`);
    if (result.format && result.format.note) output.appendLine(`  clang-format: ${result.format.note}`);
    else if (result.format && result.format.clean) output.appendLine('  clang-format: clean');

    if (result.wiring.length > 1) {
        output.appendLine('');
        output.appendLine(`Wiring checklist for ${result.wiring.length} robots: `
            + result.wiring.map((w) => w.robot).join(', '));
        output.appendLine('  The generated files are shared by every robot, but each');
        output.appendLine("  robot's control file needs its own registration.");
    }

    for (const plan of result.wiring) {
        output.appendLine('');
        output.appendLine(`TODO -- wire into ${path.relative(cwd, plan.control_file)}`);
        if (plan.tag) output.appendLine(`  (this robot's registration functions use the tag \`${plan.tag}\`)`);
        for (const step of plan.steps) {
            const where = step.line ? `line ${step.line}` : 'location not found';
            output.appendLine('');
            output.appendLine(`  ${step.n}. ${where}  ${step.where}`
                + (step.already_present ? '  [already present]' : ''));
            for (const line of step.snippet.split('\n')) output.appendLine(`       ${line}`);
            if (step.note) output.appendLine(`       ^ ${step.note}`);
        }
        for (const w of plan.warnings) {
            output.appendLine('');
            output.appendLine(`  ! ${w}`);
        }
    }
    output.appendLine('');
    output.appendLine(`Undo: ${result.undo}`);
}

module.exports = { activate, deactivate };
