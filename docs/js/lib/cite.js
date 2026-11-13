// Citation dialog.
import { h } from './ui.js';

const CITATION = 'I. Cuiral-Zueco and G. López-Nicolás, "Multiscale Procrustes-Based 3-D Shape Control," '
    + 'IEEE/ASME Transactions on Mechatronics, vol. 29, no. 3, pp. 1738-1748, 2024. doi: 10.1109/TMECH.2023.3325934.';
const BIBTEX = [
    '@article{cuiral2024multiscale,',
    '  author  = {Cuiral-Zueco, Ignacio and L{\\\'o}pez-Nicol{\\\'a}s, Gonzalo},',
    '  title   = {Multiscale {P}rocrustes-Based 3-{D} Shape Control},',
    '  journal = {IEEE/ASME Transactions on Mechatronics},',
    '  year    = {2024},',
    '  volume  = {29},',
    '  number  = {3},',
    '  pages   = {1738--1748},',
    '  doi     = {10.1109/TMECH.2023.3325934},',
    '}',
].join('\n');

async function copy(text, button) {
    try {
        await navigator.clipboard.writeText(text);
    } catch {
        const area = h('textarea', { style: 'position:fixed;opacity:0' });
        area.value = text;
        document.body.append(area);
        area.select();
        document.execCommand('copy');
        area.remove();
    }
    button.textContent = 'Copied ✓';
    setTimeout(() => { button.textContent = 'Copy'; }, 1400);
}

export function citeDialog() {
    const block = (title, text) => {
        const copyButton = h('button', { class: 'button small', type: 'button' }, 'Copy');
        copyButton.addEventListener('click', () => copy(text, copyButton));
        return h('div', { class: 'cite-block' }, h('div', { class: 'cite-block-head' }, h('span', {}, title), copyButton), h('pre', {}, text));
    };
    const close = h('button', { class: 'button', type: 'button' }, 'Close');
    const dialog = h('dialog', { class: 'cite-dialog', 'aria-labelledby': 'citeTitle' },
        h('p', { class: 'kicker' }, 'Citation'),
        h('h2', { id: 'citeTitle' }, 'Cite the paper'),
        h('p', { class: 'cite-note' }, h('a', { href: 'https://doi.org/10.1109/TMECH.2023.3325934', target: '_blank', rel: 'noopener' }, 'doi.org/10.1109/TMECH.2023.3325934')),
        block('Reference', CITATION),
        block('BibTeX', BIBTEX),
        h('div', { class: 'cite-actions' }, close));
    close.addEventListener('click', () => dialog.close());
    dialog.addEventListener('click', (event) => {           // a click on the backdrop closes
        const r = dialog.getBoundingClientRect();
        if (event.clientX < r.left || event.clientX > r.right || event.clientY < r.top || event.clientY > r.bottom) dialog.close();
    });
    dialog.addEventListener('close', () => dialog.remove());
    document.body.append(dialog);
    dialog.showModal();
}
