/** @type {import('tailwindcss').Config} */
export default {
  content: ['./src/**/*.{astro,html,js,jsx,md,mdx,svelte,ts,tsx,vue}'],
  darkMode: 'class',
  theme: {
    extend: {
      colors: {
        primary: {
          50: '#eff6ff',
          100: '#dbeafe',
          200: '#bfdbfe',
          300: '#93c5fd',
          400: '#60a5fa',
          500: '#3b82f6',
          600: '#2563eb',
          700: '#1d4ed8',
          800: '#1e40af',
          900: '#1e3a8a',
          950: '#172554',
        },
      },
      typography: (theme) => ({
        DEFAULT: {
          css: {
            '--tw-prose-body': theme('colors.gray.700'),
            '--tw-prose-headings': theme('colors.gray.900'),
            '--tw-prose-links': theme('colors.primary.600'),
            '--tw-prose-code': theme('colors.gray.900'),
            '--tw-prose-pre-bg': theme('colors.gray.50'),
            maxWidth: 'none',
          },
        },
        dark: {
          css: {
            '--tw-prose-body': theme('colors.gray.300'),
            '--tw-prose-headings': theme('colors.gray.100'),
            '--tw-prose-links': theme('colors.primary.400'),
            '--tw-prose-code': theme('colors.gray.100'),
            '--tw-prose-pre-bg': theme('colors.gray.900'),
          },
        },
      }),
    },
  },
  plugins: [require('@tailwindcss/typography')],
};
