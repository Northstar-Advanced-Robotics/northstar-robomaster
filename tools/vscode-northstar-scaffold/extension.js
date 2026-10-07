/*
 * NorthStar Scaffold -- status-bar buttons for generating subsystems and commands.
 *
 * Plain JavaScript on purpose: there is no Node toolchain in the devcontainer,
 * so this extension has no build step. VS Code runs it on its own bundled Node
 * and injects `require('vscode')`. It is installed by symlinking this folder
 * into ~/.vscode-server/extensions/northstar.scaffold-0.0.1 -- see
 * scripts/install-extension.sh.
 *
 * All the real work lives in scripts/scaffold (Python). This file only asks
 * for the name and folder and runs it, like the "Scaffold - New ..." tasks.
 */

const vscode = require('vscode');
const { execFile } = require('child_process');

const SCAFFOLD = 'scripts/scaffold';
const PROJECT_SRC = 'northstar-robomaster-project/src';

function activate(context) {
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

/**
 * Subsystem headers as quick-pick items: the file name (what --requires takes)
 * as the label and its folder as the description, `directory`'s own first.
 */
async function listSubsystems(directory) {
    const files = await vscode.workspace.findFiles(
        `${PROJECT_SRC}/**/*_subsystem.hpp`, '**/taproot/**', 500);
    const prefix = `${PROJECT_SRC}/control/`;
    return files
        .map((file) => {
            const rel = vscode.workspace.asRelativePath(file, false);
            const dir = rel.slice(0, rel.lastIndexOf('/'));
            return {
                label: rel.slice(rel.lastIndexOf('/') + 1, -'.hpp'.length),
                description: dir.startsWith(prefix) ? dir.slice(prefix.length) : dir,
            };
        })
        .sort((x, y) =>
            (y.description === directory) - (x.description === directory)
            || x.label.localeCompare(y.label));
}

async function wizard(kind) {
    const folder = vscode.workspace.workspaceFolders && vscode.workspace.workspaceFolders[0];
    if (!folder) {
        vscode.window.showErrorMessage('Open the northstar-robomaster folder first.');
        return;
    }
    const root = folder.uri;
    const cwd = root.fsPath;
    const example = kind === 'command' ? 'chassis_spin' : 'chassis';

    const name = await vscode.window.showInputBox({
        title: `New ${kind}`,
        prompt: `${kind === 'command' ? 'Command' : 'Subsystem'} name, without "${kind}" (e.g. ${example})`,
        validateInput: (v) =>
            /^[A-Za-z][A-Za-z0-9 _-]*$/.test(v || '')
                ? null
                : 'Start with a letter; letters, digits, spaces, _ and - only.',
    });
    if (!name) return;

    // Blank is allowed: the scaffolder then derives the folder from the name
    // (or, for a command, uses its subsystem's folder).
    const directory = await vscode.window.showInputBox({
        title: `New ${kind}`,
        prompt: 'Folder under src/control/ (e.g. chassis; leave blank to use the name)',
        validateInput: (v) =>
            /^([A-Za-z][A-Za-z0-9_/]*)?$/.test(v || '') ? null : 'Letters, digits, _ and / only.',
    });
    if (directory === undefined) return;

    const args = [kind, name];
    if (directory) args.push('--dir', directory);

    if (kind === 'command') {
        const pick = await vscode.window.showQuickPick(await listSubsystems(directory), {
            title: `New ${kind}`,
            placeHolder: 'Subsystem file name (e.g. chassis_subsystem)',
            matchOnDescription: true,
        });
        if (!pick) return;
        args.push('--requires', pick.label);
    }

    const result = await run(cwd, args);
    const header = result.files.find((f) => f.path.endsWith('.hpp'));
    if (header) {
        await vscode.window.showTextDocument(
            await vscode.workspace.openTextDocument(vscode.Uri.file(header.path)));
    }
}

module.exports = { activate, deactivate };
