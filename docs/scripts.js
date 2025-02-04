function navigateToLab(labPage) {
    window.location.href = labPage;
}

// for the sidebar collapsibles
document.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('.collapsible').forEach(item => {
        const arrow = document.createElement('span');
        arrow.classList.add('collapsible-arrow');
        arrow.innerHTML = '▶';
        item.insertBefore(arrow, item.firstChild);

        item.addEventListener('click', () => {
            const content = item.nextElementSibling;
            if (content.classList.contains('collapsible-content')) {
                const isExpanded = content.style.display === 'block';
                content.style.display = isExpanded ? 'none' : 'block';

                // rotate arrow
                arrow.style.transform = isExpanded ? 'rotate(0deg)' : 'rotate(90deg)';
            }
        });
    });
});
