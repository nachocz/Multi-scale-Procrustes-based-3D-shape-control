// Equations with KaTeX (vendored, offline), in the notation of the paper.

export const MACROS = {
    '\\R': '\\mathbb{R}',
    '\\X': '\\mathbf{X}', '\\Y': '\\mathbf{Y}', '\\Xh': '\\hat{\\mathbf{X}}', '\\Xb': '\\bar{\\mathbf{X}}', '\\Yb': '\\bar{\\mathbf{Y}}',
    '\\x': '\\mathbf{x}', '\\y': '\\mathbf{y}', '\\xh': '\\hat{\\mathbf{x}}', '\\xb': '\\bar{\\mathbf{x}}', '\\yb': '\\bar{\\mathbf{y}}',
    '\\Tm': '\\mathbf{T}', '\\Rm': '\\mathbf{R}', '\\tv': '\\mathbf{t}', '\\Hm': '\\mathbf{H}', '\\Um': '\\mathbf{U}',
    '\\Vm': '\\mathbf{V}', '\\Km': '\\mathbf{K}', '\\Sm': '\\mathbf{S}', '\\Qm': '\\mathbf{Q}', '\\Wm': '\\mathbf{W}', '\\I': '\\mathbf{I}',
    '\\uv': '\\mathbf{u}', '\\dP': 'd_{\\mathcal{P}}', '\\Pop': '\\mathcal{P}', '\\Lyap': '\\mathcal{V}',
    '\\SE': '\\mathrm{SE}(3)', '\\SO': '\\mathrm{SO}(3)', '\\Tr': '\\mathrm{Tr}', '\\T': '\\intercal', '\\rs': 'r^{*}',
};

export function tex(source, display = false) {
    return window.katex.renderToString(source, {
        displayMode: display, macros: { ...MACROS }, throwOnError: false, strict: 'ignore',
    });
}

// Render every <span class="tex"> (inline) and <div class="tex-block"> (display) below root, once.
export function renderMath(root = document) {
    for (const el of root.querySelectorAll('.tex, .tex-block')) {
        if (el.dataset.rendered) continue;
        const ref = el.dataset.eq ? `<span class="eq-ref">(${el.dataset.eq})</span>` : '';
        el.innerHTML = tex(el.textContent, el.classList.contains('tex-block')) + ref;
        el.dataset.rendered = '1';
    }
}
